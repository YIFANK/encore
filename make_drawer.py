"""Drawer cell, K=0 vs K=3 side by side, from the plain-robot replays (development seed 51)."""
import os, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
HERE = os.path.dirname(os.path.abspath(__file__)); R = f"{HERE}/src_media/replay_mp4"
F = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial Bold.ttf", 26)
def read(path, n, size):
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", path, "-vf", f"scale={size}:{size}", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], capture_output=True).stdout
    fr = np.frombuffer(raw, np.uint8).reshape(-1, size, size, 3)
    return fr[np.linspace(0, len(fr) - 1, n).round().astype(int)]
S, N = 448, 7 * 24
a = read(f"{R}/goal_open_middle_drawer_task_k0.mp4", N, S)
b = read(f"{R}/goal_open_middle_drawer_task_k3.mp4", N, S)
W, H = 2 * S + 12, S + 48
p = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "24", "-i", "-",
                      "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "24", "-movflags", "+faststart", f"{HERE}/media/drawer.mp4"], stdin=subprocess.PIPE)
for fa, fb in zip(a, b):
    c = Image.new("RGB", (W, H), (255, 255, 255))
    c.paste(Image.fromarray(fa), (0, 48)); c.paste(Image.fromarray(fb), (S + 12, 48))
    d = ImageDraw.Draw(c)
    d.text((10, 10), "K=0  ·  0/50", font=F, fill=(100, 100, 100))
    d.text((S + 22, 10), "K=3  ·  50/50", font=F, fill=(21, 101, 192))
    p.stdin.write(c.tobytes())
p.stdin.close(); p.wait()
print(os.path.getsize(f"{HERE}/media/drawer.mp4") // 1024, "KB")
