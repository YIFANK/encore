"""Photograph each AVFoundation camera index, so roles can be matched to indices.

The three cameras Heron reads over UVC are addressed by INDEX, and macOS assigns
indices by enumeration order — which depends on which port each camera is in and
on how many cameras are visible at all. Two things therefore shuffle them:
replugging in a different order, and handing one camera to librealsense (a
device seized that way vanishes from AVFoundation, and everything after it moves
down one).

So the indices in the config are not durable and there is no point trying to
remember the plug order. Re-derive it instead: this captures one frame per index
and the images say which is which.

Run in a GUI Terminal — macOS denies cameras to SSH sessions, root or not. No
sudo needed, this is the AVFoundation path:

    ~/Heron/.venv/bin/python ~/Heron/tools/identify_uvc.py

Writes /tmp/uvcid/index<N>.png plus a log.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

OUT = "/tmp/uvcid"
LOG = f"{OUT}/run.log"
WARMUP_FRAMES = 8  # the first frames off a UVC camera are black or half-exposed


def say(msg: str) -> None:
    print(msg, flush=True)
    try:
        with open(LOG, "a") as fh:
            fh.write(msg + "\n")
    except OSError:
        pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--max-index", type=int, default=6,
                    help="how many indices to probe")
    ap.add_argument("--settle-s", type=float, default=0.6)
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        print("opencv is not installed in this interpreter", flush=True)
        return 2
    os.makedirs(OUT, exist_ok=True)
    say(f"--- {time.strftime('%Y-%m-%d %H:%M:%S')} ---")

    found = 0
    for idx in range(args.max_index):
        cap = cv2.VideoCapture(idx)
        if not cap.isOpened():
            cap.release()
            say(f"  index {idx}: no camera")
            continue
        frame = None
        for _ in range(WARMUP_FRAMES):
            ok, f = cap.read()
            if ok:
                frame = f
        cap.release()
        time.sleep(args.settle_s)  # let macOS release it before the next open
        if frame is None:
            say(f"  index {idx}: opened but gave no frame")
            continue
        path = f"{OUT}/index{idx}.png"
        cv2.imwrite(path, frame)
        say(f"  index {idx}: {frame.shape[1]}x{frame.shape[0]} -> {path}")
        found += 1

    say(f"{found} camera(s) captured. Images and log are in {OUT}.")
    say("Nothing else is needed from this Terminal - they can be read over SSH.")
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
