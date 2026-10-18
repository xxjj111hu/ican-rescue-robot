# -*- coding: utf-8 -*-
"""解析 STL：判断四个轮子是否同款、外形是否为带斜置辊子的麦轮，并渲染出来看。"""
import hashlib
import struct
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

MESH = Path("sim3/opt/ican_shoot_sim/share/ican_land_sim/meshes")
NAMES = ["link_left_w", "link_left_s", "link_right_w", "link_right_s"]
OUT = "说明图-四个轮子的网格.png"


def load_stl(path: Path):
    raw = path.read_bytes()
    if raw[:5].lower() == b"solid" and b"facet" in raw[:500].lower():
        verts, tris = [], []
        for line in raw.decode("utf-8", "ignore").splitlines():
            s = line.strip().split()
            if s and s[0] == "vertex":
                verts.append([float(s[1]), float(s[2]), float(s[3])])
        v = np.array(verts)
        return v.reshape(-1, 3, 3)
    n = struct.unpack("<I", raw[80:84])[0]
    data = np.frombuffer(raw[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    f = data[:, :48].copy().view("<f4").reshape(n, 4, 3)   # 法向 + 3 顶点
    return f[:, 1:4, :].astype(np.float64)


def font(size):
    for p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
        try:
            return ImageFont.truetype(p, size)
        except Exception:
            pass
    return ImageFont.load_default()


def render(tris, view, size=360, pad=14):
    """view: 视线方向（投影时丢弃的轴索引）"""
    keep = [i for i in range(3) if i != view]
    p = tris[:, :, keep]
    depth = tris[:, :, view].mean(axis=1)
    order = np.argsort(depth)
    lo = p.reshape(-1, 2).min(axis=0)
    hi = p.reshape(-1, 2).max(axis=0)
    span = np.maximum(hi - lo, 1e-9)
    k = (size - 2 * pad) / span.max()
    off = (np.array([size, size]) - span * k) / 2.0

    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1e-9
    n = n / ln[:, None]
    light = np.array([0.4, 0.5, 0.76])
    light = light / np.linalg.norm(light)
    shade = np.clip(n @ light, 0, 1)

    img = Image.new("RGB", (size, size), (22, 24, 28))
    d = ImageDraw.Draw(img)
    for i in order:
        pts = [(float((p[i, j, 0] - lo[0]) * k + off[0]),
                float((p[i, j, 1] - lo[1]) * k + off[1])) for j in range(3)]
        c = int(60 + 170 * shade[i])
        d.polygon(pts, fill=(c, int(c * 0.95), int(c * 0.9)))
    return img


info = []
hashes = {}
for nm in NAMES:
    f = MESH / f"{nm}.STL"
    tris = load_stl(f)
    hashes[nm] = hashlib.sha256(f.read_bytes()).hexdigest()[:12]
    bb = tris.reshape(-1, 3)
    lo, hi = bb.min(axis=0), bb.max(axis=0)
    dim = (hi - lo) * 1000.0        # mm
    axle = int(np.argmin(dim))      # 最薄的轴 = 轮轴方向
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1)
    ln[ln == 0] = 1e-9
    n = n / ln[:, None]
    area = 0.5 * ln
    ang = np.degrees(np.arccos(np.clip(np.abs(n[:, axle]), 0, 1)))
    tot = area.sum()
    band = area[(ang > 25) & (ang < 65)].sum() / tot * 100
    info.append((nm, hashes[nm], dim, axle, band, len(tris)))
    print(f"{nm}: sha={hashes[nm]} 尺寸(mm)={np.round(dim,1)} 轮轴={'XYZ'[axle]} "
          f"斜置面占比={band:.1f}% 三角面={len(tris)}")

print("四个轮子网格是否完全相同:", len(set(hashes.values())) == 1)

# 出图：上排正视(沿轮轴看)，下排俯视
views = {"正视（沿轮轴看）": None, "俯视（从上往下看）": 2}
S = 360
cols = len(NAMES)
sheet = Image.new("RGB", (S * cols + 20 * (cols + 1), (S + 44) * 2 + 60), (12, 13, 16))
d = ImageDraw.Draw(sheet)
d.text((20, 14), "四个轮子的网格（来自 ican_land_sim/meshes，同一把尺子渲染）", font=font(24), fill=(255, 255, 255))

for row, (label, _) in enumerate(views.items()):
    d.text((20, 46 + row * (S + 44) + 4), label, font=font(20), fill=(255, 214, 102))
for col, nm in enumerate(NAMES):
    tris = load_stl(MESH / f"{nm}.STL")
    bb = tris.reshape(-1, 3)
    dim = (bb.max(axis=0) - bb.min(axis=0)) * 1000.0
    axle = int(np.argmin(dim))
    for row, view in enumerate([axle, 2]):
        img = render(tris, view, S)
        cap = f"{nm}" + (f"  {dim[0]:.0f}×{dim[1]:.0f}×{dim[2]:.0f}mm" if row == 0 else "")
        box = Image.new("RGB", (S, S + 44), (18, 19, 22))
        ImageDraw.Draw(box).text((8, S + 12), cap, font=font(16), fill=(230, 230, 230))
        box.paste(img, (0, 0))
        sheet.paste(box, (20 + col * (S + 20), 72 + row * (S + 44)))

sheet.save(OUT)
print("saved", OUT, sheet.size)
