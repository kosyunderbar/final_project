# -*- coding: utf-8 -*-
"""제공 맵 도면 생성기.

단위 mm. 원점은 맵 왼쪽 위 모서리, x 오른쪽, y 아래 (위에서 내려다본 그림).
실측값을 고치면 이 파일의 '실측' 칸만 바꾸고 다시 실행한다:
    PYTHONIOENCODING=utf-8 python 제공맵_도면/build_map.py
만드는 것: 제공맵.svg, 제공맵_4096.png, 제공맵_치수도.svg/.pdf/.png, 제공맵_좌표.json
"""
import json
import math
import pathlib
import subprocess

OUT = pathlib.Path(__file__).resolve().parent
CHROME = r"C:\Program Files\Google\Chrome\Application\chrome.exe"

# ── 실측 (사용자, 2026-11-24) ──────────────────────────────
MAP = 1000            # 맵 한 변
TRACK_OUT = 720       # 검은 경로 바깥 가장자리 한 변
TRACK_W = 20          # 경로 폭
TRACK_IN = 680        # 경로 안쪽 가장자리 한 변
ZONE = 150            # 색 구역 한 변
CORNER = 150          # 모서리 곡선 구역 (바깥 모서리에서 150 × 150)
LEFT_GAP = 100        # 왼쪽 갈림길: 직선과 가장 벌어진 곳 (바깥 가장자리끼리)
RIGHT_IN_GAP = 150    # 오른쪽 갈림길: 안쪽으로 가장 벌어진 곳 (바깥 가장자리끼리)
RIGHT_OUT_GAP = 150   # 오른쪽 갈림길: 바깥쪽으로 가장 벌어진 곳 (바깥 가장자리끼리)

# ── 사진에서 추정 (실측 필요) ───────────────────────────────
ZONES = [  # id, 이름, 중심(x, y), 역할(공정 배정 확정), 색
    ("yellow", "노랑", (850, 500), "출발", "#F2D94B"),
    ("orange", "주황", (500, 150), "P1 프레스", "#EC9459"),
    ("green", "초록", (150, 325), "P2 차체", "#6CBF96"),
    ("purple", "보라", (150, 675), "P3 도장", "#5B3F9F"),
    ("blue", "파랑", (325, 850), "P4 의장", "#2E64C6"),
    ("red", "빨강", (675, 850), "검사", "#D5503F"),
]
LEFT_Y = (390, 585)   # 왼쪽 갈림길이 갈라지고 합류하는 y
MAT = "#F6F7F5"
INK = "#1B1C1F"
LOGO = "#8D96AB"

# ── 계산 ────────────────────────────────────────────────
HALF = TRACK_W / 2
C0 = (MAP - TRACK_OUT) / 2 + HALF          # 중심선 사각형 시작 (150)
C1 = MAP - C0                              # 끝 (850)
R = CORNER - HALF                          # 모서리 중심선 반지름 (140)
assert TRACK_OUT - 2 * TRACK_W == TRACK_IN, "실측값끼리 맞지 않음"


def arc_pt(cx, cy, deg, r=R):
    a = math.radians(deg)
    return (cx + r * math.cos(a), cy + r * math.sin(a))


# 갈림길 중심선이 지나는 점 (사진에서 읽은 모양 + 실측 벌어짐)
LX = C0 + LEFT_GAP - TRACK_W                # 왼쪽 갈림길 가장 먼 중심선 x (230)
RIX = C1 - (RIGHT_IN_GAP - TRACK_W)        # 오른쪽 안쪽 가장 먼 중심선 x (720)
ROX = C1 + (RIGHT_OUT_GAP - TRACK_W)       # 오른쪽 바깥 가장 먼 중심선 x (980)
y0, y1 = LEFT_Y
LEFT_PTS = [(C0, y0), (C0 + 42, y0 + 23), (LX - 8, y0 + 57), (LX, y0 + 90),
            (LX - 9, y0 + 126), (C0 + 40, y0 + 162), (C0, y1)]
RS = arc_pt(C1 - R, C0 + R, -45)           # 오른쪽 위 모서리 45° 지점에서 갈라짐
RE = arc_pt(C1 - R, C1 - R, 15)            # 오른쪽 아래 모서리 시작 부근에서 합류
ROD = ROX - C1                             # 바깥쪽 중심선 벌어짐
RIGHT_PTS = [RS, (774, 236), (736, 292), (RIX, 345), (727, 402), (763, 455),
             (C1, 500), (C1 + 0.69 * ROD, 548), (ROX, 618), (C1 + 0.89 * ROD, 674), (C1 + 0.45 * ROD, 716), RE]


def catmull(pts):
    """점들을 지나는 매끈한 곡선 → 3차 베지어 조각 목록."""
    p = [(2 * pts[0][0] - pts[1][0], 2 * pts[0][1] - pts[1][1])] + list(pts) + \
        [(2 * pts[-1][0] - pts[-2][0], 2 * pts[-1][1] - pts[-2][1])]
    segs = []
    for i in range(1, len(p) - 2):
        a, b, c, d = p[i - 1], p[i], p[i + 1], p[i + 2]
        c1 = (b[0] + (c[0] - a[0]) / 6, b[1] + (c[1] - a[1]) / 6)
        c2 = (c[0] - (d[0] - b[0]) / 6, c[1] - (d[1] - b[1]) / 6)
        segs.append((b, c1, c2, c))
    return segs


def bez_d(segs):
    f = lambda q: f"{q[0]:.1f} {q[1]:.1f}"
    d = f"M{f(segs[0][0])}"
    for _, c1, c2, e in segs:
        d += f" C{f(c1)} {f(c2)} {f(e)}"
    return d


def bez_sample(segs, n=24):
    out = []
    for k, (b, c1, c2, e) in enumerate(segs):
        for i in range(0 if k == 0 else 1, n + 1):
            t = i / n
            u = 1 - t
            out.append((u**3 * b[0] + 3 * u * u * t * c1[0] + 3 * u * t * t * c2[0] + t**3 * e[0],
                        u**3 * b[1] + 3 * u * u * t * c1[1] + 3 * u * t * t * c2[1] + t**3 * e[1]))
    return out


def main_loop_d():
    a, b, r = C0, C1, R
    return (f"M{b} {a + r} L{b} {b - r} A{r} {r} 0 0 1 {b - r} {b} L{a + r} {b} "
            f"A{r} {r} 0 0 1 {a} {b - r} L{a} {a + r} A{r} {r} 0 0 1 {a + r} {a} "
            f"L{b - r} {a} A{r} {r} 0 0 1 {b} {a + r} Z")


def main_loop_ccw(step=10):
    """노랑(오른쪽 가운데)에서 출발해 반시계(위에서 볼 때)로 한 바퀴."""
    a, b, r = C0, C1, R
    pts = []

    def line(p, q):
        n = max(1, round(math.dist(p, q) / step))
        pts.extend((p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n) for i in range(n))

    def arc(cx, cy, d0, d1):
        n = max(1, round(abs(math.radians(d1 - d0)) * r / step))
        pts.extend(arc_pt(cx, cy, d0 + (d1 - d0) * i / n) for i in range(n))

    line((b, MAP / 2), (b, a + r)); arc(b - r, a + r, 0, -90)
    line((b - r, a), (a + r, a)); arc(a + r, a + r, -90, -180)
    line((a, a + r), (a, b - r)); arc(a + r, b - r, 180, 90)
    line((a + r, b), (b - r, b)); arc(b - r, b - r, 90, 0)
    line((b, b - r), (b, MAP / 2))
    pts.append((b, MAP / 2))
    return pts


LEFT = catmull(LEFT_PTS)
RIGHT = catmull(RIGHT_PTS)
FONT = "'Pretendard','Noto Sans KR','Malgun Gothic','Apple SD Gothic Neo',sans-serif"


def map_group(labels=False):
    s = [f'<rect x="0" y="0" width="{MAP}" height="{MAP}" fill="{MAT}"/>']
    for zid, name, (cx, cy), role, col in ZONES:
        s.append(f'<rect id="zone-{zid}" x="{cx - ZONE / 2}" y="{cy - ZONE / 2}" width="{ZONE}" height="{ZONE}" fill="{col}"/>')
    s.append(f'<g fill="{LOGO}" font-family="{FONT}" text-anchor="middle">'
             f'<text x="{MAP / 2}" y="484" font-size="31" font-weight="800" letter-spacing="0.5">SSAFY</text>'
             f'<text x="{MAP / 2}" y="509" font-size="13.5" font-weight="700">IoT Data 물류로봇</text></g>')
    s.append(f'<g fill="none" stroke="{INK}" stroke-width="{TRACK_W}" stroke-linejoin="round">'
             f'<path id="track-main" d="{main_loop_d()}"/>'
             f'<path id="track-left-branch" d="{bez_d(LEFT)}"/>'
             f'<path id="track-right-branch" d="{bez_d(RIGHT)}"/></g>')
    if labels:
        pos = {"yellow": (892, 452), "orange": (500, 104), "green": (107, 331),
               "purple": (107, 681), "blue": (325, 902), "red": (675, 902)}
        s.append(f'<g font-family="{FONT}" font-size="17" font-weight="700" text-anchor="middle" fill="{INK}" fill-opacity=".72">')
        for zid, name, _, _, _ in ZONES:
            x, y = pos[zid]
            s.append(f'<text x="{x}" y="{y}">{name}</text>')
        s.append('</g>')
    return "\n".join(s)


def clean_svg():
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {MAP} {MAP}" '
            f'width="{MAP}mm" height="{MAP}mm">\n<title>제공 맵 (1000 × 1000 mm)</title>\n'
            f'{map_group()}\n</svg>\n')


# ── 치수도 ──────────────────────────────────────────────
W, H = 1840, 1300
OX, OY = 250, 200
EST = "#C9561A"


def dim_h(x1, x2, y, ref, label, est=False, above=True):
    col = EST if est else "#27313B"
    ty = y - 7 if above else y + 21
    ext = "".join(f'<line class="e" x1="{x}" y1="{ref}" x2="{x}" y2="{y + (-8 if y < ref else 8)}"/>' for x in (x1, x2))
    return (ext + f'<line class="d" x1="{x1}" y1="{y}" x2="{x2}" y2="{y}" marker-start="url(#ar)" marker-end="url(#ar)"/>'
            f'<text class="t" x="{(x1 + x2) / 2}" y="{ty}" fill="{col}">{label}</text>')


def dim_v(y1, y2, x, ref, label, est=False, left=True):
    col = EST if est else "#27313B"
    tx = x - 8 if left else x + 20
    ext = "".join(f'<line class="e" x1="{ref}" y1="{y}" x2="{x + (-8 if x < ref else 8)}" y2="{y}"/>' for y in (y1, y2))
    cy = (y1 + y2) / 2
    return (ext + f'<line class="d" x1="{x}" y1="{y1}" x2="{x}" y2="{y2}" marker-start="url(#ar)" marker-end="url(#ar)"/>'
            f'<text class="t" x="{tx}" y="{cy}" fill="{col}" transform="rotate(-90 {tx} {cy})">{label}</text>')


def chain_h(xs, y, ref, ests, above=True):
    return "".join(dim_h(a, b, y, ref, f"{b - a:g}", e, above) for a, b, e in zip(xs, xs[1:], ests))


def chain_v(ys, x, ref, ests, left=True):
    return "".join(dim_v(a, b, x, ref, f"{b - a:g}", e, left) for a, b, e in zip(ys, ys[1:], ests))


def zone(zid):
    return next(z for z in ZONES if z[0] == zid)


def dims():
    o, z = (MAP - TRACK_OUT) / 2, ZONE / 2
    s = []
    # 위: 주황 / 경로 / 전체
    ox = zone("orange")[2][0]
    s.append(chain_h([0, ox - z, ox + z, MAP], -45, 0, [True, False, True]))
    s.append(chain_h([0, o, MAP - o, MAP], -100, 0, [False, False, False]))
    s.append(dim_h(0, MAP, -155, 0, f"{MAP}"))
    # 왼쪽: 초록·보라 / 경로 / 전체
    gy, py = zone("green")[2][1], zone("purple")[2][1]
    s.append(chain_v([0, gy - z, gy + z, py - z, py + z, MAP], -45, 0, [True, False, True, False, True]))
    s.append(chain_v([0, o, MAP - o, MAP], -100, 0, [False, False, False]))
    s.append(dim_v(0, MAP, -155, 0, f"{MAP}"))
    # 아래: 파랑·빨강
    bx, rx = zone("blue")[2][0], zone("red")[2][0]
    s.append(chain_h([0, bx - z, bx + z, rx - z, rx + z, MAP], MAP + 45, MAP, [True, False, True, False, True], above=False))
    # 오른쪽: 노랑
    yy = zone("yellow")[2][1]
    s.append(chain_v([0, yy - z, yy + z, MAP], MAP + 45, MAP, [True, False, True], left=False))
    # 안쪽 680
    s.append(dim_h(C0 + HALF, C1 - HALF, 650, 650, f"안쪽 {TRACK_IN}"))
    # 경로 폭 20
    s.append(f'<line class="d" x1="500" y1="{C1 - HALF}" x2="500" y2="{C1 + HALF}"/>'
             f'<line class="e" x1="488" y1="{C1 - HALF}" x2="512" y2="{C1 - HALF}"/>'
             f'<line class="e" x1="488" y1="{C1 + HALF}" x2="512" y2="{C1 + HALF}"/>'
             f'<path class="d" d="M500 {C1 - HALF - 4} L500 {C1 - 110} L520 {C1 - 110}"/>'
             f'<text class="t" x="524" y="{C1 - 104}" style="text-anchor:start" fill="#27313B">경로 폭 {TRACK_W}</text>')
    # 모서리 곡선 구역 (왼쪽 위)
    s.append(f'<rect x="{o}" y="{o}" width="{CORNER}" height="{CORNER}" fill="none" stroke="#2A7FD4" stroke-width="2" stroke-dasharray="8 6"/>')
    s.append(dim_h(o, o + CORNER, o - 22, o, f"{CORNER}"))
    s.append(f'<text class="t" x="{o + CORNER + 14}" y="{o + CORNER + 34}" style="text-anchor:start" fill="#2A7FD4">곡선 구역 {CORNER} × {CORNER}</text>')
    # 갈림길 벌어짐
    s.append(dim_h(C0 - HALF, LX + HALF, LEFT_Y[0] + 90, LEFT_Y[0] + 90, f"{LEFT_GAP}"))
    s.append(dim_h(RIX - HALF, C1 + HALF, 345, 345, f"{RIGHT_IN_GAP}"))
    s.append(dim_h(C1 - HALF, ROX + HALF, 618, 618, f"{RIGHT_OUT_GAP}", above=False))
    # 교차점
    s.append(f'<circle cx="{C1}" cy="500" r="17" fill="none" stroke="#2A7FD4" stroke-width="2.5"/>')
    # 원점
    s.append('<g stroke="#27313B" stroke-width="2" fill="#27313B"><path d="M0 0 L60 0" marker-end="url(#ax)"/>'
             '<path d="M0 0 L0 60" marker-end="url(#ax)"/></g>'
             '<text class="s" x="66" y="22">x</text><text class="s" x="8" y="80">y</text>')
    return "\n".join(s)


def panel():
    X = 1385
    rows = []
    t = lambda x, y, txt, cls="p", extra="": rows.append(f'<text class="{cls}" x="{x}" y="{y}" {extra}>{txt}</text>')
    t(X, 70, "제공 맵 도면", "h1")
    t(X, 102, "SSAFY IoT Data 물류로봇 맵 · 위에서 본 그림")
    t(X, 128, f"단위 mm · {MAP} × {MAP} · 2026-11-24 실측 기준")
    # 범례
    rows.append(f'<rect x="{X}" y="150" width="425" height="72" rx="6" fill="#F1F4F7"/>')
    t(X + 16, 178, "1000", "pnum", 'fill="#27313B"'); t(X + 78, 178, "검정 숫자 = 실측 (사용자 측정)")
    t(X + 16, 208, "1000", "pnum", f'fill="{EST}"'); t(X + 78, 208, "주황 숫자 = 사진에서 추정, 실측 필요")
    # 색 구역 표
    t(X, 262, f"색 구역 · {ZONE} × {ZONE} (실측)", "h2")
    t(X + 44, 290, "색", "ph"); t(X + 150, 290, "중심 x, y (추정)", "ph"); t(X + 318, 290, "공정 배정", "ph")
    y = 318
    for zid, name, (cx, cy), role, col in ZONES:
        rows.append(f'<rect x="{X}" y="{y - 17}" width="28" height="22" rx="3" fill="{col}"/>')
        t(X + 44, y, name); t(X + 150, y, f"{cx}, {cy}", "pnum", f'fill="{EST}"'); t(X + 318, y, role)
        y += 30
    # 경로
    t(X, 520, "경로", "h2")
    for i, line in enumerate([
        f"바깥 {TRACK_OUT} · 안쪽 {TRACK_IN} · 폭 {TRACK_W} (실측)",
        f"중심선은 {C0:g}~{C1:g} 사이 {C1 - C0:g} × {C1 - C0:g} 사각형",
        f"모서리 곡선 구역 {CORNER} × {CORNER} (실측)",
        f"→ 바깥 가장자리 반지름 {CORNER}, 중심선 {R:g}로 해석",
    ]):
        t(X, 548 + i * 26, line)
    # 갈림길
    t(X, 680, "갈림길", "h2")
    for i, line in enumerate([
        "벌어짐은 바깥 가장자리끼리 잰 값으로 해석",
        f"왼쪽: 최대 {LEFT_GAP} (실측)",
        f"   y {LEFT_Y[0]}에서 갈라져 y {LEFT_Y[1]}에서 합류 (추정)",
        "오른쪽 S자: 오른쪽 위 모서리에서 갈라짐",
        f"   → 안쪽으로 최대 {RIGHT_IN_GAP} (실측)",
        f"   → 노랑 가운데 ({C1:g}, 500)에서 직선과 교차",
        f"   → 바깥으로 최대 {RIGHT_OUT_GAP} (실측)",
        "   → 오른쪽 아래 모서리 시작에서 합류",
    ]):
        t(X, 708 + i * 26, line)
    # AGV 크기 비교 (같은 축척)
    t(X, 940, "같은 축척 크기 비교", "h2")
    zy = 985
    rows.append(f'<rect x="{X}" y="{zy}" width="{ZONE}" height="{ZONE}" fill="{zone("yellow")[4]}"/>')
    t(X + ZONE / 2, zy + ZONE + 26, f"색 구역 {ZONE}×{ZONE}", "p", 'text-anchor="middle"')
    ax = X + ZONE + 40
    rows.append(f'<rect x="{ax}" y="{zy}" width="195" height="230" rx="10" fill="#3A4652"/>'
                f'<rect x="{ax + 80}" y="{zy - 8}" width="35" height="18" rx="3" fill="#252D35"/>')
    t(ax + 97.5, zy + 122, "AGV", "pw", 'text-anchor="middle"')
    t(ax + 97.5, zy + 146, "195 × 230", "pw", 'text-anchor="middle"')
    t(ax + 97.5, zy - 16, "앞 (카메라)", "p", 'text-anchor="middle"')
    t(ax + 97.5, zy + 256, "JETANK 실측 (팔 제외)", "p", 'text-anchor="middle"')
    t(X, 1282, "원본: AGV_맵.png · 생성: build_map.py · 좌표: 제공맵_좌표.json", "s")
    return "\n".join(rows)


def dim_svg():
    return f'''<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">
<title>제공 맵 치수도</title>
<defs>
<marker id="ar" viewBox="0 0 10 10" refX="10" refY="5" markerWidth="9" markerHeight="9" markerUnits="userSpaceOnUse" orient="auto-start-reverse"><path d="M0 1.5 L10 5 L0 8.5z" fill="#27313B"/></marker>
<marker id="ax" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="12" markerHeight="12" markerUnits="userSpaceOnUse" orient="auto"><path d="M0 1 L10 5 L0 9z" fill="#27313B"/></marker>
<style>
text{{font-family:{FONT}}}
.d{{stroke:#27313B;stroke-width:1.4;fill:none}}
.e{{stroke:#8B96A1;stroke-width:1}}
.t{{font-size:19px;font-weight:600;text-anchor:middle;font-variant-numeric:tabular-nums}}
.s{{font-size:15px;fill:#5D6B78}}
.h1{{font-size:34px;font-weight:800;fill:#1B1C1F}}
.h2{{font-size:20px;font-weight:700;fill:#1B1C1F}}
.p{{font-size:17px;fill:#27313B}}
.ph{{font-size:15px;fill:#5D6B78}}
.pnum{{font-size:17px;font-weight:600;font-variant-numeric:tabular-nums}}
.pw{{font-size:17px;font-weight:700;fill:#FFFFFF}}
</style></defs>
<rect width="{W}" height="{H}" fill="#FFFFFF"/>
<g transform="translate({OX} {OY})">
{map_group(labels=True)}
<rect x="0" y="0" width="{MAP}" height="{MAP}" fill="none" stroke="#9AA5B0" stroke-width="1.5"/>
{dims()}
</g>
<line x1="1350" y1="40" x2="1350" y2="1270" stroke="#DCE2E8" stroke-width="1.5"/>
{panel()}
</svg>
'''


def geometry():
    r1 = lambda p: [round(p[0], 1), round(p[1], 1)]
    return {
        "name": "제공 맵 (SSAFY IoT Data 물류로봇)",
        "unit": "mm",
        "frame": "원점 = 맵 왼쪽 위 모서리, x 오른쪽, y 아래 (위에서 내려다본 그림). "
                 "가운데 원점 cm로 바꾸려면 u = (x - 500) / 10, v = (500 - y) / 10 (위쪽이 +).",
        "measured_2026_11_24": {"map": MAP, "track_outer": TRACK_OUT, "track_inner": TRACK_IN,
                                "track_width": TRACK_W, "zone": ZONE, "corner_zone": CORNER,
                                "left_branch_gap": LEFT_GAP, "right_branch_inner_gap": RIGHT_IN_GAP,
                                "right_branch_outer_gap": RIGHT_OUT_GAP},
        "estimated_from_photo": ["zones[].center", "branch join/split points",
                                 "branch curve shape"],
        "interpretation": {"corner": f"바깥 가장자리 반지름 {CORNER}, 중심선 반지름 {R:g}",
                           "branch_gap": "직선과 갈림길의 바깥 가장자리끼리 잰 거리"},
        "map": {"size": [MAP, MAP], "mat_color": MAT},
        "track": {"width": TRACK_W, "color": INK, "centerline_square": [C0, C0, C1, C1],
                  "corner_radius_centerline": R},
        "zones": [{"id": zid, "name": name, "role": role, "center": [cx, cy], "size": ZONE,
                   "rect": [cx - ZONE / 2, cy - ZONE / 2, cx + ZONE / 2, cy + ZONE / 2], "color": col}
                  for zid, name, (cx, cy), role, col in ZONES],
        "paths_centerline": {
            "main_loop_ccw_from_yellow": [r1(p) for p in main_loop_ccw()],
            "left_branch_top_to_bottom": [r1(p) for p in bez_sample(LEFT)],
            "right_branch_top_to_bottom": [r1(p) for p in bez_sample(RIGHT)],
        },
        "junctions": [
            {"id": "left_split", "at": r1(LEFT_PTS[0])}, {"id": "left_join", "at": r1(LEFT_PTS[-1])},
            {"id": "right_split", "at": r1(RS)}, {"id": "right_cross", "at": [C1, 500]},
            {"id": "right_join", "at": r1(RE)},
        ],
    }


def chrome(args):
    subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars", *args],
                   check=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)


def main():
    (OUT / "제공맵.svg").write_text(clean_svg(), encoding="utf-8")
    (OUT / "제공맵_치수도.svg").write_text(dim_svg(), encoding="utf-8")
    (OUT / "제공맵_좌표.json").write_text(json.dumps(geometry(), ensure_ascii=False, indent=1), encoding="utf-8")

    tmp = OUT / "_render"
    tmp.mkdir(exist_ok=True)
    big = clean_svg().replace(f'width="{MAP}mm" height="{MAP}mm"', 'width="4096" height="4096"')
    (tmp / "clean.html").write_text(f'<!doctype html><meta charset="utf-8"><style>html,body{{margin:0}}svg{{display:block}}</style>{big}', encoding="utf-8")
    (tmp / "dim.html").write_text(
        '<!doctype html><meta charset="utf-8"><style>@page{size:420mm 297mm;margin:0}'
        'html,body{margin:0}svg{display:block;width:420mm;height:296.7mm}</style>' + dim_svg(), encoding="utf-8")
    (tmp / "dim_png.html").write_text(
        '<!doctype html><meta charset="utf-8"><style>html,body{margin:0}svg{display:block}</style>' + dim_svg(), encoding="utf-8")
    chrome(["--window-size=4096,4096", "--force-device-scale-factor=1",
            f"--screenshot={OUT / '제공맵_4096.png'}", (tmp / "clean.html").as_uri()])
    chrome(["--no-pdf-header-footer", f"--print-to-pdf={OUT / '제공맵_치수도.pdf'}", (tmp / "dim.html").as_uri()])
    chrome([f"--window-size={W},{H}", "--force-device-scale-factor=2",
            f"--screenshot={OUT / '제공맵_치수도.png'}", (tmp / "dim_png.html").as_uri()])
    for f in tmp.iterdir():
        f.unlink()
    tmp.rmdir()
    print("ok")


if __name__ == "__main__":
    main()
