"""Client for the SAM2 mask service (`tools/sam2_server.py`).

ER-2 gives boxes and points, never masks. A box around a bowl also contains the
table under it and whatever is behind it, so every depth statistic taken over the
box is computed partly from things that are not the bowl. Dropping points at or
below the support plane removes the table; it cannot remove the object standing
behind the bowl, nor tell the bowl's outer wall from the table seen through its
opening. A mask can.

The segmenter is optional by construction. If `sam_url` is unset, unreachable,
or slow, `mask()` returns None and the caller falls back to the box — the same
accuracy we had this morning, never an outage. A perception component that can
take the robot down when it fails is worse than not having it.
"""
from __future__ import annotations

import base64
import io
import json
import logging
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Optional

import numpy as np

_log = logging.getLogger(__name__)

DEFAULT_TIMEOUT_S = 8.0
# Below this SAM2 self-score the mask is not worth trusting over the box; in
# practice a confident mask scores well above 0.9 and a confused one collapses.
MIN_SCORE = 0.60
# A mask covering nearly the whole box has segmented the box, not the object,
# and one covering almost none of it has latched onto a speck.
MIN_BOX_FRACTION = 0.02
MAX_BOX_FRACTION = 0.98
# Point-prompted, there is no box to measure against. A single graspable object
# never fills a third of an overhead frame; the table does.
MAX_FRAME_FRACTION = 0.33
# Consecutive failures after which we stop paying the timeout on every call...
FAILURES_BEFORE_MUTE = 3
# ...and how long that silence lasts. Muting used to be permanent: `enabled`
# went false, which stopped the requests, which meant nothing could ever
# succeed, which meant `_failures` never reset. One slow minute disabled masks
# for the rest of the episode and grounding quietly fell back to single-pixel
# deprojection — the mode measured at 22.7 mm against the mask's 11.0 mm. It
# still ran, and still reported success.
MUTE_SECONDS = 60.0


class SamSegmenter:
    """Turn a detection box into a mask, or return None and let the caller cope."""

    def __init__(self, url: Optional[str], timeout_s: float = DEFAULT_TIMEOUT_S,
                 log=None) -> None:
        self.url = url.rstrip("/") if url else None
        self.timeout_s = float(timeout_s)
        self.log = log
        self._failures = 0
        self._muted_until = 0.0
        self.calls = 0
        self.masks = 0
        # Both the etk box and the robot workstation reach the outside world
        # through an HTTP proxy, and urllib would helpfully send a request for
        # 127.0.0.1 there too. A local service must never go through a proxy.
        self._opener = (urllib.request.build_opener(urllib.request.ProxyHandler({}))
                        if _is_local(self.url) else urllib.request.build_opener())

    @property
    def enabled(self) -> bool:
        if self.url is None:
            return False
        if self._failures < FAILURES_BEFORE_MUTE:
            return True
        if time.time() >= self._muted_until:
            self._failures = 0          # cooldown over: give it another chance
            return True
        return False

    def candidates(self, rgb: np.ndarray, box=None, point=None) -> list[dict]:
        """Every reading SAM2 offers for this prompt, best-scoring first.

        From a point there are three — the part, the object, the group — and the
        highest score is not reliably the one wanted: pointed at a laptop, SAM2
        scored the keyboard highest. Callers that know the camera calibration
        can tell these apart by physical size; `mask()` is for those that
        cannot, and just takes the top one.
        """
        return self._request(rgb, box, point)

    def mask(self, rgb: np.ndarray, box=None, point=None) -> Optional[np.ndarray]:
        """Boolean HxW mask for one object, from a box, a point, or both.

        Either prompt alone works, which is not a convenience: on the lab's wide
        overhead frame ER-2 points at everything reliably and boxes almost
        nothing, so point-only is the ONLY route to a mask there. `point` also
        disambiguates when a box holds two touching objects.
        """
        got = self._request(rgb, box, point)
        return got[0]["mask"] if got else None

    def _request(self, rgb: np.ndarray, box, point) -> list[dict]:
        """One /segment call. Returns surviving candidates, best-scoring first."""
        if not self.enabled:
            return []
        if box is None and point is None:
            raise ValueError("segmentation needs a box, a point, or both")
        self.calls += 1
        t0 = time.perf_counter()
        norm = None
        if box is not None:
            u0, v0, u1, v1 = (float(x) for x in box)
            norm = [min(u0, u1), min(v0, v1), max(u0, u1), max(v0, v1)]
        try:
            image_b64, scale = _encoded(rgb)
            # The prompt travels with the image, so it travels in its pixels.
            sent_box = [v * scale for v in norm] if norm is not None else None
            sent_pt = ([float(point[0]) * scale, float(point[1]) * scale]
                       if point is not None else None)
            payload = json.dumps({
                "image": image_b64,
                "box": sent_box,
                "point": sent_pt,
            }).encode()
            req = urllib.request.Request(
                f"{self.url}/segment", data=payload,
                headers={"Content-Type": "application/json"})
            with self._opener.open(req, timeout=self.timeout_s) as r:
                body = json.loads(r.read())
        except (urllib.error.URLError, OSError, ValueError, json.JSONDecodeError) as e:
            self._failures += 1
            muted = self._failures >= FAILURES_BEFORE_MUTE
            if muted:
                self._muted_until = time.time() + MUTE_SECONDS
            self._event("sam_unavailable", error=f"{type(e).__name__}: {e}",
                        consecutive=self._failures, muted=muted,
                        retry_in_s=round(MUTE_SECONDS) if muted else 0)
            return []
        self._failures = 0
        if not body.get("ok"):
            self._event("sam_error", error=str(body.get("error"))[:200])
            return []

        # The plausibility test is "how much of the region did it claim?", and
        # what counts as the region depends on what we prompted with. Against a
        # box, a mask filling all of it segmented the box rather than the
        # object. Against a point there is no such reference, so the frame is
        # the only yardstick, and the bound that matters is the upper one: a
        # point on a table happily returns the whole table.
        if norm is not None:
            area = max(1.0, (norm[2] - norm[0] + 1) * (norm[3] - norm[1] + 1))
            lo, hi, what = MIN_BOX_FRACTION, MAX_BOX_FRACTION, "box"
        else:
            area = float(rgb.shape[0] * rgb.shape[1])
            lo, hi, what = 0.0, MAX_FRAME_FRACTION, "frame"

        raw = body.get("candidates") or [{"mask": body["mask"],
                                          "score": body.get("score", 0.0)}]
        kept, rejected = [], []
        for c in raw:
            mask = _restore(_b64_png_mask(c["mask"]), rgb.shape[:2])
            score = float(c.get("score", 0.0))
            fraction = float(mask.sum()) / area
            if score < MIN_SCORE or not (lo <= fraction <= hi):
                # Say which test failed. "SAM rejected" sends whoever reads the
                # log looking at the wrong thing half the time.
                rejected.append({
                    "score": round(score, 3), "fraction": round(fraction, 3),
                    "reason": ("low score" if score < MIN_SCORE else
                               f"covers {fraction:.0%} of the {what}"
                               if fraction > hi else "a speck")})
                continue
            kept.append({"mask": mask, "score": score, "pixels": int(mask.sum()),
                         "fraction": fraction})
        if not kept:
            self._event("sam_mask_rejected", relative_to=what, rejected=rejected)
            return []
        self.masks += 1
        self._event("sam_mask", score=round(kept[0]["score"], 3),
                    pixels=kept[0]["pixels"], kept=len(kept), rejected=len(rejected),
                    fraction=round(kept[0]["fraction"], 3), relative_to=what,
                    cached=bool(body.get("cached")), server_ms=body.get("ms"),
                    ms=round(1000 * (time.perf_counter() - t0), 1))
        return kept

    def propose(self, rgb: np.ndarray, max_masks: int = 40) -> list[dict]:
        """Every object-shaped region in the image, largest first, or [].

        Each entry is {"mask": HxW bool, "box": (u0,v0,u1,v1), "pixels": int,
        "score": float}. This asks nothing about what the objects ARE, which is
        the point: a detector that fails to recognise a 22 px stick of butter
        returns no box for it, while its region is plainly there to be found.
        """
        if not self.enabled:
            return []
        t0 = time.perf_counter()
        try:
            image_b64, scale = _encoded(rgb)
            payload = json.dumps({"image": image_b64,
                                  "max_masks": int(max_masks)}).encode()
            req = urllib.request.Request(
                f"{self.url}/propose", data=payload,
                headers={"Content-Type": "application/json"})
            with self._opener.open(req, timeout=max(self.timeout_s, 60.0)) as r:
                body = json.loads(r.read())
        except (urllib.error.URLError, OSError, ValueError, json.JSONDecodeError) as e:
            self._failures += 1
            self._event("sam_unavailable", error=f"{type(e).__name__}: {e}",
                        consecutive=self._failures, endpoint="propose")
            return []
        self._failures = 0
        if not body.get("ok"):
            self._event("sam_error", error=str(body.get("error"))[:200],
                        endpoint="propose")
            return []
        out = []
        inv = 1.0 / scale if scale else 1.0
        for m in body.get("masks", []):
            out.append({"mask": _restore(_b64_png_mask(m["mask"]), rgb.shape[:2]),
                        "box": tuple(int(round(x * inv)) for x in m["box"]),
                        "pixels": int(m.get("pixels", 0)),
                        "score": float(m.get("score", 0.0))})
        self._event("sam_proposals", count=len(out), server_ms=body.get("ms"),
                    ms=round(1000 * (time.perf_counter() - t0), 1))
        return out

    def _event(self, name: str, **kw) -> None:
        if self.log is not None:
            self.log.event(name, **kw)
        else:
            _log.debug("%s %s", name, kw)


def _is_local(url: Optional[str]) -> bool:
    if not url:
        return False
    host = urllib.parse.urlsplit(url).hostname or ""
    return host in ("localhost", "127.0.0.1", "::1", "0.0.0.0")


# The masks must stay lossless, but the IMAGE being segmented does not: SAM2
# looks at a photograph, not at exact byte values. A 1280x720 frame is over a
# megabyte as PNG and about a tenth of that as JPEG, and the measured cost of a
# call was 8 s of which 0.07 s was GPU — essentially all of it transport.
JPEG_QUALITY = 85
# THE LINK IS THE COST, NOT THE GPU. Measured on the rig 2026-08-06: one
# /propose returned server_ms 963 after 11.3 s of wall clock, and a timed
# upload put the tunnel to the GPU at 19 KB/s. A 1280x720 frame is ~240 KB
# encoded, which is twelve seconds of waiting for one second of work, six
# times an episode. SAM finds object-sized regions, and an object 60 px
# across at full size is still 30 px at half — plenty of boundary for a
# centroid. Send fewer pixels; scale the answer back up.
MAX_SEND_PX = 640


def _encoded(rgb: np.ndarray) -> tuple[str, float]:
    """(base64 JPEG, scale) — scale is sent_px / original_px."""
    from PIL import Image  # noqa: PLC0415  (only needed when the service is on)

    img = Image.fromarray(np.ascontiguousarray(rgb).astype(np.uint8))
    scale = min(1.0, MAX_SEND_PX / max(img.width, img.height))
    if scale < 1.0:
        img = img.resize((max(1, round(img.width * scale)),
                          max(1, round(img.height * scale))), Image.BILINEAR)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=JPEG_QUALITY)
    return base64.b64encode(buf.getvalue()).decode(), scale


def _png_b64(rgb: np.ndarray) -> str:
    """Kept for callers that do not care about the scale factor."""
    return _encoded(rgb)[0]


def _restore(mask: np.ndarray, shape: tuple[int, int]) -> np.ndarray:
    """A mask measured on the downscaled image, back at the frame's own size."""
    from PIL import Image  # noqa: PLC0415

    if mask.shape[:2] == shape:
        return mask
    big = Image.fromarray((np.asarray(mask) > 0).astype(np.uint8) * 255).resize(
        (shape[1], shape[0]), Image.NEAREST)
    return np.asarray(big) > 127


def _b64_png_mask(b64: str) -> np.ndarray:
    from PIL import Image  # noqa: PLC0415

    img = Image.open(io.BytesIO(base64.b64decode(b64)))
    return np.asarray(img.convert("L")) > 127
