# -*- coding: utf-8 -*-
"""상황별 주행(situ.py) 모의 시험: 직선 -> 반지름 14cm 왼쪽 90° 모서리 -> 직선.
로봇과 **같은 클래스(situ.SituDriver)** 를 돌린다. 차체 중심이 선에서 얼마나 벗어나는지 본다(안쪽 -, 바깥 +).
    python3 tools/sim_situ.py

'실제 차' 모델: 모터가 늦게 따라온다(tau초). 제자리 회전 속도·직진 속도·카메라 눈금·출발 자세를 하나씩 흔든다.
카메라 모델: 차체 13.6~24.9cm 앞, 좌우는 화면 끝(가까운 줄에서 왼쪽 9.1cm / 오른쪽 10.4cm)까지.
            아랫줄 선은 화면 왼쪽 끝에서 잘리면 '잘림'으로, 12px(약 1cm)도 안 남으면 못 본 것으로.
갈림길·그림자·반사광·렌즈 휨은 없다 — 통과해도 매트에서 된다는 뜻은 아니다.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from common import cm_per_s, load_config  # noqa
import situ  # noqa

R = 14.0
ROW_X = [13.6, 14.1, 14.5, 15.1, 15.6, 16.2, 16.8, 17.4, 18.2, 18.9, 19.7, 20.6, 21.5, 22.7, 23.7, 24.9]   # 추적 줄의 바닥 거리(실측 보정)
YL0, YR0 = 9.1, -10.4            # 맨 아랫줄에서 보이는 왼쪽/오른쪽 끝(cm)


def path_pts(ds=0.05):
    P = []
    for s in np.arange(-60, 0, ds):
        P.append((s, 0.0, 0.0))
    for a in np.arange(0, math.pi / 2, ds / R):
        P.append((R * math.sin(a), R * (1 - math.cos(a)), a))
    for s in np.arange(0, 60, ds):
        P.append((R, R + s, math.pi / 2))
    return np.array(P)


PATH = path_pts()
ARC0 = 1200                      # 호 시작 번호
ARC1 = ARC0 + int(math.pi / 2 * R / 0.05)


def deviation(x, y):
    """선까지 거리(cm). + = 바깥(진행 방향의 오른쪽), - = 안쪽"""
    d = np.hypot(PATH[:, 0] - x, PATH[:, 1] - y)
    i = int(d.argmin())
    px, py, pa = PATH[i]
    nx, ny = -math.sin(pa), math.cos(pa)
    return -((x - px) * nx + (y - py) * ny), i


def camera(x, y, th, i0, k=1.0):
    """(fp 바닥 점 목록, yb, 잘림?) — 추적기처럼 아래 줄부터 이어지는 점만"""
    c, s = math.cos(th), math.sin(th)
    seg = PATH[i0:i0 + 1400]
    dx, dy = seg[:, 0] - x, seg[:, 1] - y
    bx, by = c * dx + s * dy, -s * dx + c * dy
    fp = []
    yb, clipped = None, False
    for n, xr in enumerate(ROW_X):
        f = bx - xr
        idx = np.flatnonzero((f[:-1] < 0) & (f[1:] >= 0))
        if idx.size == 0:
            if fp:
                break
            continue
        j = idx[0]
        yy = float(by[j])
        left, right = YL0 + 0.53 * (xr - 13.6), YR0 - 0.71 * (xr - 13.6)
        beta = seg[j, 2] - th
        hw = 1.0 / max(0.2, math.cos(beta))                 # 2cm 선이 가로로 잘린 폭의 절반
        if n == 0:                                           # 아랫줄 전용 검출기: 매트 검사 없음, 왼쪽 끝에서 잘림
            lo, hi = yy - hw, min(yy + hw, left)
            if yy > right and hi - lo >= 1.04:
                yb, clipped = (lo + hi) / 2.0 * k, hi >= left - 1e-6
        if not (right + hw + 0.5 < yy < left - hw - 0.5):    # 추적기: 양옆에 매트가 보여야 함
            if fp:
                break
            continue
        fp.append((xr, yy * k))
    return fp, yb, clipped


def run(cfg, v_scale=1.0, w_deg=13.0, tau=0.2, k=1.0, e0=0.0, psi0=0.0, start=-25.0, T=150.0, verbose=False):
    drv = situ.SituDriver(cfg, cm_per_s)
    v_motor = cfg['drive']['speed']
    dt = 0.05
    x, y, th = start, -e0, -math.radians(psi0)
    vv = ww = 0.0
    left = right = 0.0
    t = 0.0
    i0 = 0
    devs = []
    fault = None
    npiv = 0
    prev_mode = ''
    corner_ok = True
    while t < T:
        # 실제 차: 명령 -> (v, w), 늦게 따라옴
        if left * right < 0:
            vc, wc = 0.0, math.copysign(math.radians(w_deg), right - left)
        else:
            vl, vr = cm_per_s(cfg, left) if left > 0 else 0.0, cm_per_s(cfg, right) if right > 0 else 0.0
            vc = (vl + vr) / 2.0 * v_scale
            wc = math.radians((right - left) / 0.30 * 9.5)          # 실측: 좌우 차 0.30(0.25/0.55)에서 8~11°/s
        vv += (vc - vv) * dt / tau
        ww += (wc - ww) * dt / tau
        x += vv * math.cos(th) * dt
        y += vv * math.sin(th) * dt
        th += ww * dt
        t += dt
        dev, i = deviation(x, y)
        i0 = max(i0, i)
        devs.append((i0, dev))
        fp, yb, clipped = camera(x, y, th, max(0, i0 - 20), k)
        found = len(fp) >= 5
        e = psi = 0.0
        if found:
            near = [p for p in fp if p[0] <= 18.0]
            ab = situ.fit_line(near if len(near) >= 4 else fp)
            e, psi = ab[0], math.atan(ab[1])

        def yb_fn(y_prev, y_min, prefer=None, _yb=yb, _cl=clipped):
            if _yb is None or _yb < y_min:
                return None, False
            return _yb, _cl
        left, right, mode, fault = drv.step(t, found, fp if found else None, e, psi, yb_fn, 'none', corner_ok, v_motor)
        if drv.corner_started:
            corner_ok = False
        if mode == 'COR_TURN' and prev_mode != 'COR_TURN':
            npiv += 1
        prev_mode = mode
        if verbose and int(round(t / dt)) % 5 == 0:
            print('%5.1f %-9s x=%5.1f y=%5.1f th=%4.0f dev=%+5.2f yb=%s L%.2f R%.2f' % (
                t, mode, x, y, math.degrees(th), dev, None if yb is None else round(yb, 1), left, right))
        if fault or y > R + 35:
            break
    devs = np.array(devs)
    arc = devs[(devs[:, 0] >= ARC0 - 60) & (devs[:, 0] <= ARC1 + 100), 1]          # 호 3cm 전 ~ 5cm 뒤
    post = devs[(devs[:, 0] > ARC1 + 100) & (devs[:, 0] <= ARC1 + 500), 1]         # 호 뒤 5~25cm
    post = post if len(post) else np.array([9.0])
    return dict(arc_in=float(arc.min()), arc_out=float(arc.max()), post_in=float(post.min()), post_out=float(post.max()),
                npiv=npiv, t=t, fault=fault, done=y > R + 35)


def show(name, r):
    print('%-34s 호 안쪽 %+.2f 바깥 %+.2f | 호 뒤 5~25cm %+.2f ~ %+.2f | 회전 %d번 %3.0f초 %s' % (
        name, r['arc_in'], r['arc_out'], r['post_in'], r['post_out'], r['npiv'], r['t'],
        '완주' if r['done'] else ('이상: %s' % r['fault'] if r['fault'] else '못 끝냄')))


if __name__ == '__main__':
    cfg = load_config()
    cases = [('기본', {}),
             ('회전 9°/s', dict(w_deg=9)), ('회전 17°/s', dict(w_deg=17)),
             ('모터 늦음 0.1초', dict(tau=0.1)), ('모터 늦음 0.3초', dict(tau=0.3)),
             ('카메라 눈금 +15%', dict(k=1.15)), ('카메라 눈금 -15%', dict(k=0.85)),
             ('속도 +20%', dict(v_scale=1.2)), ('속도 -20%', dict(v_scale=0.8)),
             ('출발 방향 +3°', dict(psi0=3)), ('출발 방향 -3°', dict(psi0=-3)),
             ('출발 옆 +1.5cm', dict(e0=1.5)), ('출발 옆 -1.5cm', dict(e0=-1.5)),
             ('나쁜 조합 1', dict(w_deg=17, tau=0.3, k=1.15, psi0=3)), ('나쁜 조합 2', dict(w_deg=9, tau=0.1, k=0.85, psi0=-3))]
    worst_in, worst_out = 0.0, 0.0
    for name, kw in cases:
        r = run(cfg, **kw)
        show(name, r)
        worst_in, worst_out = min(worst_in, r['arc_in'], r['post_in']), max(worst_out, r['arc_out'], r['post_out'])
    print('가장 나쁜 값: 안쪽 %.2fcm, 바깥 %.2fcm' % (worst_in, worst_out))
