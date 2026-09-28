"""Build the project-page videos from sealed-evaluation rollout GIFs (sim only)."""
import os, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
HERE = os.path.dirname(os.path.abspath(__file__)); SRC = f"{HERE}/src_media"; OUT = f"{HERE}/media"
F_B = "/System/Library/Fonts/Supplemental/Arial Bold.ttf"; F_R = "/System/Library/Fonts/Supplemental/Arial.ttf"
FPS = 24

def frames(gif, n, crop=None, size=None):
    im = Image.open(f"{SRC}/{gif}"); N = im.n_frames
    out = []
    for k in np.linspace(0, N - 1, n).round().astype(int):
        im.seek(int(k)); fr = im.convert("RGB")
        if crop: fr = fr.crop(crop)
        if size: fr = fr.resize(size, Image.LANCZOS)
        out.append(fr)
    return out

def label(img, top, bottom=None, size=26):
    d = ImageDraw.Draw(img); W, H = img.size
    fb = ImageFont.truetype(F_B, size); fr = ImageFont.truetype(F_R, int(size * 0.85))
    d.rectangle([0, H - (74 if bottom else 44), W, H], fill=(0, 0, 0))
    d.text((18, H - (68 if bottom else 38)), top, font=fb, fill=(255, 255, 255))
    if bottom: d.text((18, H - 34), bottom, font=fr, fill=(210, 210, 210))
    return img

def write(path, imgs, crf=30):
    W, H = imgs[0].size
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                          "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(crf),
                          "-preset", "slow", "-movflags", "+faststart", path], stdin=subprocess.PIPE)
    for im in imgs: p.stdin.write(im.tobytes())
    p.stdin.close(); p.wait()

W, H = 960, 540
# ---- teaser montage: LIBERO-PRO then RoboDojo, 2.5 s per clip
LIB = [("goal_open_middle_drawer_task_k3__ep10_ok.gif", "open the bottom drawer of the cabinet"),
       ("goal_put_cream_cheese_in_bowl_task_k3__ep10_ok.gif", "put the wine bottle in the bowl"),
       ("goal_put_wine_on_rack_task_k3__ep10_ok.gif", "put the cream cheese on the rack"),
       ("spa_bowl_on_stove_pos_k3__ep10_ok.gif", "pick up the black bowl on the stove and place it on the plate")]
RD = [("build_tower__ep10_ok.gif", "build a tower using the wooden blocks and boards", (100, 0, 540, 330)),
      ("make_kong__ep10_ok.gif", "make a kong", (90, 20, 550, 365)),
      ("classify_objects__ep14_ok.gif", "classify the objects into the bins", (90, 0, 550, 345)),
      ("stack_blocks__ep10_ok.gif", "stack three blocks", (150, 130, 510, 400)),
      ("swap_T__ep10_ok.gif", "swap the two T blocks", (180, 160, 480, 385)),
      ("insert_tubes__ep13_ok.gif", "insert the tubes into the rack", (130, 10, 510, 295))]
clips = []
for g, s in LIB:
    for fr in frames(g, int(2.5 * FPS), size=(H - 44, H - 44)):
        c = Image.new("RGB", (W, H), (0, 0, 0)); c.paste(fr, ((W - fr.width) // 2, 0))
        clips.append(label(c, "LIBERO-PRO", f"“{s}”", 24))
for g, s, box in RD:
    for fr in frames(g, int(2.5 * FPS), crop=box, size=(W, H)):
        clips.append(label(fr, "RoboDojo (bimanual)", f"“{s}”", 24))
write(f"{OUT}/teaser.mp4", clips)

# ---- drawer cell: K=0 vs K=3 side by side
S = 432
a = frames("goal_open_middle_drawer_task_k0__ep10_fail.gif", 6 * FPS, size=(S, S))
b = frames("goal_open_middle_drawer_task_k3__ep10_ok.gif", 6 * FPS, size=(S, S))
pair = []
for fa, fb in zip(a, b):
    c = Image.new("RGB", (2 * S + 12, S + 46), (255, 255, 255))
    c.paste(fa, (0, 46)); c.paste(fb, (S + 12, 46))
    d = ImageDraw.Draw(c); f = ImageFont.truetype(F_B, 24)
    d.text((10, 10), "K=0  ·  0/50", font=f, fill=(90, 90, 90))
    d.text((S + 22, 10), "K=3  ·  50/50", font=f, fill=(21, 101, 192))
    pair.append(c)
write(f"{OUT}/drawer.mp4", pair, crf=26)

# ---- RoboDojo 3x3 loop
G = [("build_tower__ep10_ok.gif", "build tower", (130, 20, 510, 305)),
     ("make_kong__ep10_ok.gif", "make kong", (110, 40, 530, 355)),
     ("classify_objects__ep14_ok.gif", "classify objects", (110, 20, 530, 335)),
     ("put_bottles_into_dustbin__ep12_ok.gif", "bottles to dustbin", (90, 20, 550, 365)),
     ("press_by_number__ep10_ok.gif", "press buttons by number", (130, 70, 510, 355)),
     ("stack_blocks__ep10_ok.gif", "stack three blocks", (180, 150, 500, 390)),
     ("swap_T__ep10_ok.gif", "swap two T blocks", (200, 180, 460, 375)),
     ("push_T__ep11_ok.gif", "push T onto outline", (260, 165, 540, 375)),
     ("insert_tubes__ep13_ok.gif", "insert tubes", (150, 30, 490, 285))]
cw, ch = 320, 240; n = 6 * FPS
tiles = [frames(g, n, crop=box, size=(cw, ch)) for g, _, box in G]
grid = []
for t in range(n):
    c = Image.new("RGB", (3 * cw + 8, 3 * ch + 8), (255, 255, 255))
    for i, (g, lab, _) in enumerate(G):
        fr = tiles[i][t].copy(); d = ImageDraw.Draw(fr); f = ImageFont.truetype(F_B, 17)
        d.rectangle([0, 0, d.textlength(lab, font=f) + 14, 26], fill=(0, 0, 0))
        d.text((7, 4), lab, font=f, fill=(255, 255, 255))
        c.paste(fr, ((i % 3) * (cw + 4), (i // 3) * (ch + 4)))
    grid.append(c)
write(f"{OUT}/robodojo.mp4", grid, crf=28)
for f in sorted(os.listdir(OUT)):
    print(f, os.path.getsize(f"{OUT}/{f}") // 1024, "KB")
