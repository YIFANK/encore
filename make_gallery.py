"""Scrolling gallery rows: RoboDojo on top (4:3 tiles), LIBERO-PRO below (square tiles), all 192 px high."""
import glob, os, random, subprocess
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
lib = sorted(glob.glob(f"{HERE}/src_media/replay_mp4/*_k3.mp4"))
rd = sorted(glob.glob(f"{HERE}/teaser_remotion/public/clips/rd_*.mp4"))
random.seed(5); random.shuffle(lib)
H, N, GAP = 192, 6 * 24, 4
def read(path, w, h):
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-vf", f"scale={w}:{h}:force_original_aspect_ratio=increase,crop={w}:{h}",
                          "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True).stdout
    fr = np.frombuffer(raw, np.uint8).reshape(-1, h, w, 3)
    return fr[np.linspace(0, len(fr) - 1, N).round().astype(int)]
def row(paths, w, out):
    tiles = [read(p, w, H) for p in paths]
    W = len(tiles) * (w + GAP)          # trailing gap so two copies tile seamlessly
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "24", "-i", "-",
                          "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", "-preset", "slow", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    for t in range(N):
        c = np.full((H, W, 3), 255, np.uint8)
        for i, tl in enumerate(tiles):
            c[:, i * (w + GAP):i * (w + GAP) + w] = tl[t]
        p.stdin.write(c.tobytes())
    p.stdin.close(); p.wait()
    return W
os.makedirs(f"{HERE}/media/gallery", exist_ok=True)
print("rd", row(rd, 256, f"{HERE}/media/gallery/row0.mp4"))
lib = lib + lib[:1]                      # 59 cells -> 60 tiles, 5 rows of 12
for r in range(5):
    print(r + 1, row(lib[r * 12:(r + 1) * 12], 192, f"{HERE}/media/gallery/row{r + 1}.mp4"))
