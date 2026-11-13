"""Smart-tag pod v0.2 — open-top box + magnetic lid, ESP32 NodeMCU-32 (51 x 25.4) raised for F-F jumpers.
x = length (USB end x=0 -> antenna end), y = width (y=W is the reader side), z = up.
"""
import os, struct, json, sys
import numpy as np
from manifold3d import Manifold

OUT = sys.argv[1] if len(sys.argv) > 1 else "stl2"
os.makedirs(OUT, exist_ok=True)

L, W = 61.0, 49.0
HW = 39.6                 # box wall top (lid underside)
LID_T = 2.0
WY0, WY1 = 1.6, W - 1.2   # inner wall faces (reader-side wall 1.2)
OY0, OX1 = -0.8, L + 0.8  # outer faces: -y wall and antenna-end wall thickened outward to 2.4 (inner space unchanged)
XEND = L - 1.6            # inner face of +x wall
BOARD = dict(x0=5.2, y0=4.1, L=51.0, W=25.4, pcb_bot=24.5, pcb_t=1.6)
ROWS = (BOARD["y0"] + 1.27, BOARD["y0"] + 1.27 + 22.86)      # pin rows (pitch 22.86)
PLASTIC_BOT = BOARD["pcb_bot"] - 2.5                          # header plastic bottom 22.0
SLOT_DEPTH = 6.5
TAG = dict(x0=14.0, x1=57.0, gap=6.7)                         # keytag pocket (42 x 35 x <=6 tag)
MAG = dict(d=6.3, depth=3.2, top=36.6, xc=8.6, yc=(5.6, 43.8))  # 6x3 magnets
HOOKS_Y = (8.0, 36.0)
CAP = dict(x0=0.8, x1=3.8, z1=HW - 0.3)
CLR = 0.25

def box(x0, y0, z0, x1, y1, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

def cyl(cx, cy, z0, h, r, seg=56):
    return Manifold.cylinder(h, r, r, seg).translate((cx, cy, z0))

def hull(pts):
    return Manifold.hull_points([list(map(float, q)) for q in pts])

# ---------------- box (floor + 3 walls, open top and open USB end) ----------------
def make_box():
    b = box(0, OY0, 0, OX1, W, HW) - box(-1, WY0, 2, XEND, WY1, HW + 1)
    b = b + box(0, WY0 + CLR, 2, 0.8, WY1 - CLR, 3.5)                    # end-cap front rib
    for (px, py) in [(20, 12), (20, 22), (42, 12), (42, 22)]:
        b = b + cyl(px, py, 2, 1.5, 1.5)                                    # tray pegs
    # keytag pocket on the reader-side wall: side rails + front lips (tag stands on the floor)
    ty0 = WY1 - TAG["gap"]; ly0 = ty0 - 1.2
    b = b + box(TAG["x0"] - 1.2, ly0, 2, TAG["x0"], WY1, HW)
    b = b + box(TAG["x1"], ly0, 2, TAG["x1"] + 1.2, WY1, HW)
    b = b + box(TAG["x0"], ly0, 2, TAG["x0"] + 3, ty0, HW)
    b = b + box(TAG["x1"] - 3, ly0, 2, TAG["x1"], ty0, HW)
    # magnet brackets (USB end corners) with 45-degree gussets -> printable without supports
    x0b, x1b = MAG["xc"] - 4, min(MAG["xc"] + 4, TAG["x0"] - 1.2)
    for side in ("-y", "+y"):
        if side == "-y":
            wy, iy = WY0, WY0 + 8.0
        else:
            wy, iy = WY1, WY1 - 8.0
        blk = box(x0b, min(wy, iy), 33.0, x1b, max(wy, iy), MAG["top"])
        gus = hull([(x, wy, 25.0) for x in (x0b, x1b)] + [(x, wy, 33.0) for x in (x0b, x1b)] + [(x, iy, 33.0) for x in (x0b, x1b)])
        b = b + blk + gus
    for yc in MAG["yc"]:
        b = b - cyl(MAG["xc"], yc, MAG["top"] - MAG["depth"], MAG["depth"] + 1, MAG["d"] / 2)
    # hook windows in the antenna-end wall
    for yc in HOOKS_Y:
        b = b - box(XEND - 0.01, yc - 2.0, 36.1, OX1 + 1, yc + 2.0, 38.1)
    return b

# ---------------- lid ----------------
def make_lid(led=(48.0, 34.0, 5.3)):
    z0 = HW
    lid = box(0, OY0, z0, OX1, W, z0 + LID_T)
    for yc in MAG["yc"]:                                                    # magnet bosses
        boss = cyl(MAG["xc"], yc, MAG["top"], z0 - MAG["top"], 5.0) ^ box(4.2, WY0 + 0.2, MAG["top"], TAG["x0"] - 1.4, WY1 - 0.2, z0)
        lid = lid + boss
    lid = lid + box(0, WY0 + CLR, z0 - 1.0, 0.8, WY1 - CLR, z0)            # ribs that trap the end cap
    lid = lid + box(4.0, WY0 + CLR, z0 - 1.0, 4.6, WY1 - CLR, z0)
    for yc in HOOKS_Y:                                                      # hooks (antenna end)
        sx1 = XEND - 0.2; sx0 = sx1 - 1.2
        lid = lid + box(sx0, yc - 1.8, z0 - 4.0, sx1, yc + 1.8, z0)
        pts = []
        for y in (yc - 1.5, yc + 1.5):
            pts += [(sx1 - 0.3, y, 36.3), (sx1 - 0.3, y, 37.8), (XEND + 1.0, y, 36.8), (XEND + 1.0, y, 37.8)]
        lid = lid + hull(pts)
    for yc in MAG["yc"]:
        lid = lid - cyl(MAG["xc"], yc, MAG["top"] - 0.6, MAG["depth"] + 0.6, MAG["d"] / 2)
    x, y, d = led
    lid = lid - cyl(x, y, z0 - 1, LID_T + 2, d / 2)
    return lid

# ---------------- end cap (USB end wall, holds the button) ----------------
def make_endcap(button="round7"):
    c = box(CAP["x0"], WY0 + CLR, 2, CAP["x1"], WY1 - CLR, CAP["z1"])
    usb_y = BOARD["y0"] + BOARD["W"] / 2; usb_z = BOARD["pcb_bot"] + BOARD["pcb_t"] + 1.5
    c = c - box(0, usb_y - 7, usb_z - 4.5, 5, usb_y + 7, usb_z + 4.5)      # USB plug opening 14 x 9
    c = c - box(0, 27, 31, CAP["x1"] - 0.4, 38, 37.5)                       # knockout: charge port (later)
    c = c - box(0, 2.5, 33, CAP["x1"] - 0.4, 9.3, 37.5)                     # knockout: power switch (later)
    by, bz = 38.5, 15.0
    if button == "round7":
        c = c - Manifold.cylinder(10, 3.5, 3.5, 48).rotate((0, 90, 0)).translate((-2, by, bz))
    elif button == "square12":
        c = c - box(-1, by - 6.2, bz - 6.2, 5, by + 6.2, bz + 6.2)
    return c

# ---------------- board tray (raised on two slotted bridges; middle pins stay free for F-F jumpers) ----------------
def make_tray(slot_w=1.4):
    z0, zf = 2.0, 3.2
    y0, y1 = ROWS[0] - 2.0, ROWS[1] + 2.0
    t = box(4.0, y0, z0, 59.0, y1, zf)
    t = t + box(4.0, y0, zf, 11.0, y1, PLASTIC_BOT)                            # USB-end bridge
    t = t + box(4.0, 8.0, zf, 5.0, 25.6, BOARD["pcb_bot"] + 0.5)               # datum (touches PCB edge)
    t = t + box(49.8, y0, zf, 58.2, y1, PLASTIC_BOT)                           # antenna-end bridge
    xe = BOARD["x0"] + BOARD["L"] + 1.0
    t = t + box(xe, 8.0, zf, xe + 1.0, 25.6, BOARD["pcb_bot"] + 0.5)           # end stop (1 mm play)
    for yc in ROWS:
        t = t - box(3, yc - slot_w / 2, PLASTIC_BOT - SLOT_DEPTH, 60, yc + slot_w / 2, PLASTIC_BOT + 1)
    for (px, py) in [(20, 12), (20, 22), (42, 12), (42, 22)]:
        t = t - cyl(px, py, 1, 5, 1.75)
    return t

# ---------------- coupons ----------------
def coupon_slots():
    c = box(0, 0, 0, 60, 12, 8)
    for i, (w, x0) in enumerate([(1.2, 2), (1.4, 22), (1.6, 42)]):
        c = c - box(x0, 6 - w / 2, 8 - SLOT_DEPTH, x0 + 16, 6 + w / 2, 9)
        for k in range(i + 1):
            c = c - cyl(x0 + 3 + k * 2.5, 10.2, 7.4, 1, 0.6, 24)
    return c

def coupon_nfc_steps():
    c = None
    for i, t in enumerate([0.8, 1.2, 1.6, 2.0]):
        x0 = (i % 2) * 46; y0 = (i // 2) * 36
        cell = box(x0, y0, 0, x0 + 46, y0 + 36, t)
        c = cell if c is None else c + cell
    return c

def coupon_spacer_frame():
    return box(0, 0, 0, 92, 61, 10) - box(2, 2, -1, 90, 59, 11)

def coupon_magnet():
    c = box(0, 0, 0, 36, 12, 5)
    for i, d in enumerate([6.1, 6.3, 6.5]):
        c = c - cyl(6 + i * 12, 6, 5 - 3.2, 4, d / 2)
        for k in range(i + 1):
            c = c - cyl(3 + i * 12 + k * 2.5, 1.4, 4.4, 1, 0.5, 24)
    return c

def coupon_height_frames():
    """stackable frames 5 / 5 / 10 mm (outer 52 x 46) -> read distance without a ruler"""
    out = None
    for i, h in enumerate([5.0, 5.0, 10.0]):
        f = box(0, 0, 0, 52, 46, h) - box(1.6, 1.6, -1, 50.4, 44.4, h + 1)
        for k in range(int(h // 5)):            # 1 or 2 notches on top edge = 5 or 10 mm
            f = f - box(6 + k * 4, -1, h - 1.0, 8 + k * 4, 2.6, h + 1)
        f = f.translate((0, i * 50, 0))
        out = f if out is None else out + f
    return out

# ---------------- dummies for clearance checks ----------------
def dummies():
    b = BOARD
    pcb = box(b["x0"], b["y0"], b["pcb_bot"], b["x0"] + b["L"], b["y0"] + b["W"], b["pcb_bot"] + b["pcb_t"])
    module = box(b["x0"] + b["L"] - 18, b["y0"] + 3.7, b["pcb_bot"] + b["pcb_t"], b["x0"] + b["L"], b["y0"] + 21.7, 29.2)
    usb = box(b["x0"] - 0.6, b["y0"] + 8.9, 26.1, b["x0"] + 5, b["y0"] + 16.5, 29.0)
    plastic = None; pins = None
    for yc in ROWS:
        pl = box(b["x0"] + 1.3, yc - 1.25, PLASTIC_BOT, b["x0"] + b["L"] - 1.3, yc + 1.25, b["pcb_bot"])
        plastic = pl if plastic is None else plastic + pl
        for k in range(19):
            px = b["x0"] + 2.64 + k * 2.54
            pn = box(px - 0.32, yc - 0.32, PLASTIC_BOT - 6.0, px + 0.32, yc + 0.32, PLASTIC_BOT)
            pins = pn if pins is None else pins + pn
    tag = box(14.5, WY1 - TAG["gap"] + 0.35, 2.0, 56.5, WY1 - 0.35, 37.0)
    reserve = box(15, 3, 29.5, 45, 31, HW - 0.2)
    return dict(pcb=pcb + module + usb + plastic, pins=pins, tag=tag, battery_reserve=reserve)

# ---------------- output ----------------
def write_stl(m, path):
    me = m.to_mesh()
    v = np.asarray(me.vert_properties)[:, :3].astype(np.float32)
    f = np.asarray(me.tri_verts).astype(np.int64)
    tri = v[f]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True); ln[ln == 0] = 1
    rec = np.zeros(len(f), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"] = (n / ln).astype(np.float32); rec["v"] = tri
    with open(path, "wb") as fh:
        fh.write(b"smarttag pod v0.2".ljust(80, b" ")); fh.write(struct.pack("<I", len(f))); fh.write(rec.tobytes())

def on_bed(m, rot=None):
    if rot:
        m = m.rotate(rot)
    x0, y0, z0, _, _, _ = m.bounding_box()
    return m.translate((-x0, -y0, -z0))

if __name__ == "__main__" or True:
    BOX, LID, CAPm, TRAY = make_box(), make_lid(), make_endcap("round7"), make_tray(1.4)
    files = {
        "pod2_box.stl": on_bed(BOX),
        "pod2_lid.stl": on_bed(LID, (180, 0, 0)),                     # top face on the bed
        "pod2_endcap_btn_round7.stl": on_bed(CAPm, (0, 90, 0)),
        "pod2_endcap_btn_square12.stl": on_bed(make_endcap("square12"), (0, 90, 0)),
        "pod2_endcap_no_button.stl": on_bed(make_endcap("none"), (0, 90, 0)),
        "test1_slot_width.stl": on_bed(coupon_slots()),
        "test2_nfc_wall_steps.stl": on_bed(coupon_nfc_steps()),
        "test3_nfc_spacer_10mm.stl": on_bed(coupon_spacer_frame()),
        "test4_magnet_pocket.stl": on_bed(coupon_magnet()),
        "test5_height_frames_5_5_10.stl": on_bed(coupon_height_frames()),
    }
    for sw in (1.2, 1.4, 1.6):
        files[f"pod2_tray_slot{sw}.stl"] = on_bed(make_tray(sw))
    info = {}
    for name, m in files.items():
        if m.is_empty():
            raise SystemExit("EMPTY " + name)
        write_stl(m, os.path.join(OUT, name))
        bb = m.bounding_box()
        info[name] = dict(size=[round(bb[3] - bb[0], 1), round(bb[4] - bb[1], 1), round(bb[5] - bb[2], 1)],
                          vol_cm3=round(m.volume() / 1000, 2), genus=m.genus())
    parts = dict(box=BOX, lid=LID, endcap=CAPm, tray=TRAY)
    parts.update(dummies())
    names = list(parts); chk = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if {a, b} <= {"pcb", "pins", "tag", "battery_reserve"} and {a, b} != {"pcb", "battery_reserve"}:
                continue
            v = (parts[a] ^ parts[b]).volume()
            chk[f"{a} x {b}"] = round(v, 3)
    info["_interference_mm3"] = chk
    json.dump(info, open(os.path.join(OUT, "_info.json"), "w"), indent=1, ensure_ascii=False)
    for k, v in info.items():
        print(k, v)
