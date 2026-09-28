"""Rollout wall for the page and the teaser title: sealed K=3 successes, LIBERO-PRO + RoboDojo."""
import glob, os, random, subprocess
import numpy as np
from PIL import Image
HERE = os.path.dirname(os.path.abspath(__file__))
lib = sorted(glob.glob(f"{HERE}/src_media/replay_mp4/*_k3.mp4"))
rd = sorted(glob.glob(f"{HERE}/teaser_remotion/public/clips/rd_*.mp4"))
random.seed(3)
def read(path, n, size):
    """n frames resampled over the clip, center-cropped to a square of `size`."""
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-vf", f"scale={size}:{size}:force_original_aspect_ratio=increase,crop={size}:{size}",
                          "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True).stdout
    fr = np.frombuffer(raw, np.uint8).reshape(-1, size, size, 3)
    return fr[np.linspace(0, len(fr) - 1, n).round().astype(int)]
def wall(paths, cols, rows, size, n, out, gap=2):
    tiles = [read(p, n, size) for p in paths]
    W, H = cols * size + (cols - 1) * gap, rows * size + (rows - 1) * gap
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "24", "-i", "-",
                          "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "30", "-preset", "slow", "-movflags", "+faststart", out], stdin=subprocess.PIPE)
    for t in range(n):
        c = np.full((H, W, 3), 255, np.uint8)
        for i, tl in enumerate(tiles):
            r, q = divmod(i, cols); c[r * (size + gap):r * (size + gap) + size, q * (size + gap):q * (size + gap) + size] = tl[t]
        p.stdin.write(c.tobytes())
    p.stdin.close(); p.wait()
# page wall: 10 x 6
pick = random.sample(lib, 51) + rd[:7] + rd[:2]
random.shuffle(pick)
wall(pick, 10, 6, 192, 6 * 24, f"{HERE}/media/wall.mp4")
# teaser title background clips: 24 single clips copied as wall_XX
tt = random.sample(lib, 18) + rd[:6]; random.shuffle(tt)
for k, pth in enumerate(tt):
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-i", pth, "-vf", "scale=320:320:force_original_aspect_ratio=increase,crop=320:320,fps=24,format=yuv420p",
                    "-c:v", "libx264", "-crf", "26", "-an", f"{HERE}/teaser_remotion/public/clips/wall_{k:02d}.mp4"])
print(os.path.getsize(f"{HERE}/media/wall.mp4") // 1024, "KB")
