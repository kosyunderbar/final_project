# -*- coding: utf-8 -*-
"""구역에 세워 놓고 색 값을 재서 hsv_calib.json에 저장한다 (가이드: 들어가는 자세·가운데·나가는 자세 세 번)

AGV(컨테이너)에서, 카메라를 다른 프로그램이 잡고 있지 않을 때:
    python3 calib_hsv.py yellow in          # 구역 앞(들어가는 자세)
    python3 calib_hsv.py yellow center
    python3 calib_hsv.py yellow out
    python3 calib_hsv.py --show             # 지금까지 잰 값과 색 사이 거리

저장한 사진으로(카메라 없이):
    python3 calib_hsv.py yellow center --from-dir logs/frames_xxx --frames 10
"""
import argparse
import glob
import json
import os
import time

import cv2
import numpy as np

import color_detector as cd
from common import HSV_CALIB_PATH, LOG_DIR, load_config, open_camera, set_camera_angle


def load_calib():
    if os.path.exists(HSV_CALIB_PATH):
        with open(HSV_CALIB_PATH, encoding='utf-8') as f:
            return json.load(f)
    return {'samples': [], 'hues': {}}


def save_calib(calib):
    # 색마다 표본 H의 원 평균을 대표값으로
    by = {}
    for s in calib['samples']:
        by.setdefault(s['color'], []).append(s['h'])
    calib['hues'] = {k: round(cd.circular_mean_h(np.array(v)), 1) for k, v in by.items()}
    with open(HSV_CALIB_PATH, 'w', encoding='utf-8') as f:
        json.dump(calib, f, ensure_ascii=False, indent=1)


def show(cfg, calib):
    hues = dict(cfg['color']['hues'])
    print('색별 H (재서 바꾼 값은 *):')
    for k in cfg['route']['order']:
        mark = '*' if k in calib.get('hues', {}) else ' '
        print('  %s %-7s H=%5.1f' % (mark, k, hues[k]))
    names = list(hues)
    print('가장 가까운 두 색 사이 거리:')
    pairs = sorted(((cd.hue_dist(hues[a], hues[b]), a, b) for i, a in enumerate(names) for b in names[i + 1:]))
    for d, a, b in pairs[:3]:
        print('  %s - %s : %.1f' % (a, b, d))
    print('표본 %d개' % len(calib['samples']))


def frames_from_camera(cfg, n):
    set_camera_angle(cfg)
    cam = open_camera(cfg)
    try:
        time.sleep(1.0)
        out = []
        for _ in range(n):
            out.append(cam.value.copy())
            time.sleep(0.1)
        return out
    finally:
        cam.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('color', nargs='?')
    ap.add_argument('pose', nargs='?', default='center', choices=['in', 'center', 'out'])
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--from-dir')
    ap.add_argument('--show', action='store_true')
    a = ap.parse_args()
    cfg = load_config()
    calib = load_calib()
    if a.show or not a.color:
        show(cfg, calib)
        return
    if a.color not in cfg['route']['order']:
        raise SystemExit('색 이름은 %s 중 하나' % cfg['route']['order'])

    if a.from_dir:
        files = sorted(glob.glob(os.path.join(a.from_dir, '*.jpg')))[:a.frames]
        frames = [cv2.imread(f) for f in files]
    else:
        frames = frames_from_camera(cfg, a.frames)

    c = cfg['color']
    rs = [cd.measure(f, c) for f in frames]
    rs = [r for r in rs if r.h is not None and r.fill >= c['min_fill']]
    if not rs:
        raise SystemExit('띠 안에 색이 충분하지 않습니다 (fill < %.2f). 구역 위에 제대로 섰는지 확인하세요' % c['min_fill'])
    h = cd.circular_mean_h(np.array([r.h for r in rs]))
    s = float(np.median([r.s for r in rs]))
    v = float(np.median([r.v for r in rs]))
    fill = float(np.median([r.fill for r in rs]))
    calib['samples'].append({'color': a.color, 'pose': a.pose, 'h': round(h, 1), 's': round(s), 'v': round(v),
                             'fill': round(fill, 2), 'n': len(rs), 'at': time.strftime('%Y-%m-%d %H:%M:%S')})
    save_calib(calib)
    os.makedirs(os.path.join(LOG_DIR, 'calib'), exist_ok=True)
    cv2.imwrite(os.path.join(LOG_DIR, 'calib', '%s_%s_%s.jpg' % (a.color, a.pose, time.strftime('%H%M%S'))), frames[-1])
    print('%s/%s: H=%.1f S=%.0f V=%.0f fill=%.2f (%d장)' % (a.color, a.pose, h, s, v, fill, len(rs)))
    show(load_config(), calib)


if __name__ == '__main__':
    main()
