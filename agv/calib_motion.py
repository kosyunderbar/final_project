# -*- coding: utf-8 -*-
"""직진 속도 재기 — 자 없이 카메라로

앞(30~40cm)에 색 구역이 보이게 곧은 선 위에 놓고, 노트북에서:
    import calib_motion
    k = calib_motion.forward_speed(camera, robot, cfg)     # 구역이 다가온 만큼 = 간 거리
    calib_motion.save_speed_unit(k)                        # config.json drive.cm_s_per_unit 에 저장
"""
import json
import math
import time

import numpy as np

from common import CONFIG_PATH


def measure_psi(camera, tracker, n=7, timeout=2.0):
    """선 방향 ψ(도)의 중앙값. 못 찾으면 None"""
    vals, t0 = [], time.time()
    while len(vals) < n and time.time() - t0 < timeout:
        r = tracker.track(camera.value.copy(), 'none')
        if r.found:
            vals.append(math.degrees(r.psi))
        time.sleep(0.05)
    return float(np.median(vals)) if len(vals) >= 3 else None


def zone_near_x(img, cfg, fm, color=None):
    """앞에 보이는 색 구역의 가까운 변까지 거리(차체 중심 기준 cm). (색, cm) 또는 (None, None)"""
    import cv2
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    c = cfg['color']
    best = (0, None, None)
    for name, hue in c['hues'].items():
        if color and name != color:
            continue
        d = np.minimum(np.abs(hsv[..., 0].astype(int) - hue), 180 - np.abs(hsv[..., 0].astype(int) - hue))
        m = ((d <= 8) & (hsv[..., 1] > c['min_s']) & (hsv[..., 2] > c['min_v'])).astype(np.uint8)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        sub = m[:, 70:154]                                   # 가운데 세로 띠
        cols = [np.nonzero(sub[:, j])[0] for j in range(sub.shape[1])]
        bottoms = [col.max() for col in cols if len(col) > 10]
        if len(bottoms) > 20 and int(sub.sum()) > best[0]:
            best = (int(sub.sum()), name, float(np.median(bottoms)))
    if best[1] is None or best[2] >= img.shape[0] - 3:      # 구역이 화면 아래로 잘리면 못 잼
        return None, None
    return best[1], float(fm.to_floor([[112, best[2]]])[0][0])


def forward_speed(camera, robot, cfg, motor=None, runs=(1.0, 2.5), settle=0.6):
    """앞(30~40cm)에 색 구역이 보이게 곧은 선 위에 놓고 실행. 구역 가까운 변까지 거리가 줄어든 만큼 = 간 거리.
    짧게 한 번·길게 한 번 가서 차이로 '다 빨라진 뒤 속도'(cm/s)를 구한다(출발 때 느린 부분 빼기)."""
    from floor import FloorMap
    motor = motor or cfg['drive']['speed']
    fm = FloorMap(cfg)

    def look():
        vals, col = [], None
        for _ in range(7):
            c_, x = zone_near_x(camera.value.copy(), cfg, fm)
            if x is not None:
                vals.append(x)
                col = c_
            time.sleep(0.05)
        return (col, float(np.median(vals))) if len(vals) >= 3 else (None, None)

    dists = []
    for sec in runs:
        col, x0 = look()
        if x0 is None:
            print('앞에 색 구역이 안 보입니다 (가까운 변이 화면 안에 보이게)')
            return None
        try:
            robot.left_motor.value = robot.right_motor.value = motor
            time.sleep(sec)
        finally:
            robot.stop()
        time.sleep(settle)
        _, x1 = look()
        if x1 is None:
            print('가고 나서 구역이 안 보입니다. 구역을 더 멀리(35~40cm) 두고 다시')
            return None
        dists.append(x0 - x1)
        print('%s 구역: %.1f초 -> %.1fcm 감 (%.1f -> %.1fcm)' % (col, sec, x0 - x1, x0, x1))
    v = (dists[1] - dists[0]) / (runs[1] - runs[0])
    lag = runs[0] - dists[0] / v if v > 0 else 0.0
    print('모터 %.2f: 다 빨라진 뒤 %.2fcm/s, 출발 늦음 약 %.2f초 -> cm_s_per_unit = %.2f (지금 %.2f)'
          % (motor, v, lag, v / (motor - cfg['drive']['motor_deadband']), cfg['drive']['cm_s_per_unit']))
    return v / (motor - cfg['drive']['motor_deadband'])


def save_speed_unit(k):
    with open(CONFIG_PATH, encoding='utf-8') as f:
        c = json.load(f)
    c['drive']['cm_s_per_unit'] = round(float(k), 2)
    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        json.dump(c, f, ensure_ascii=False, indent=2)
    print('config.json drive.cm_s_per_unit =', c['drive']['cm_s_per_unit'])
