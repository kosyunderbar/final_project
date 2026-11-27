# -*- coding: utf-8 -*-
"""바닥 좌표 맞추기 (10분, 실주행 전에 한 번. 카메라 각도나 차체를 바꾸면 다시)

준비: AGV를 **곧은 선 위 가운데**에 똑바로 놓고, 앞에 색 구역(15×15cm) 하나가 화면에 통째로 보이게 한다.
      (주황 구역 앞 직선이 좋다. 구역의 가까운 변이 화면 아래 1/3쯤에 오게)
잴 것: 자로 **차체 중심(앞뒤 길이 23cm의 가운데) → 색 구역의 가까운 변**까지 거리(cm) 하나.

    python3 calib_floor.py orange 24.5          # 색, 거리(cm)
    python3 calib_floor.py orange 24.5 --image logs/snapshots/calib/xxx.jpg   # 저장한 사진으로

결과: floor_calib.json + logs/calib/floor_check.jpg (바닥 5cm 격자를 화면에 그린 것 — 선과 구역에 맞는지 눈으로 확인)
"""
import argparse
import json
import os
import time

import cv2
import numpy as np

import color_detector as cd
from common import LOG_DIR, load_config, open_camera, set_camera_angle
from floor import CALIB_PATH

ZONE_CM = 15.0


def find_zone_quad(img, hue, cfg):
    """색 구역의 네 모서리(화면 px): 가까운 왼쪽, 가까운 오른쪽, 먼 오른쪽, 먼 왼쪽"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    c = cfg['color']
    d = np.minimum(np.abs(hsv[..., 0].astype(int) - hue), 180 - np.abs(hsv[..., 0].astype(int) - hue))
    m = ((d <= 10) & (hsv[..., 1] > c['min_s']) & (hsv[..., 2] > c['min_v'])).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))   # 구역 위를 지나는 검은 선 틈을 메운다
    cs = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not cs:
        return None, m
    # 검은 선이 구역 한가운데를 지나가 구역이 두 조각으로 갈린다 -> 큰 조각들을 합친 볼록 껍질로 구역 전체를 잡는다
    big = max(cv2.contourArea(x) for x in cs)
    parts = [x for x in cs if cv2.contourArea(x) >= 0.25 * big]
    hull = cv2.convexHull(np.vstack(parts))
    quad = None
    for eps in np.linspace(0.01, 0.08, 15):
        ap = cv2.approxPolyDP(hull, eps * cv2.arcLength(hull, True), True)
        if len(ap) == 4:
            quad = ap.reshape(4, 2).astype(np.float32)
            break
    if quad is None:
        return None, m
    # 아래(가까운) 두 점, 위(먼) 두 점
    q = quad[np.argsort(quad[:, 1])]
    far = q[:2][np.argsort(q[:2, 0])]
    near = q[2:][np.argsort(q[2:, 0])]
    return np.float32([near[0], near[1], far[1], far[0]]), m


def calibrate(img, color, dist_cm, cfg=None):
    """사진 한 장 + 색 + 거리로 floor_calib.json을 만든다. 반환 (메시지, 확인 그림). 못 찾으면 (메시지, None)
    노트북에서: calib_floor.calibrate(camera.value.copy(), 'orange', 24.5)"""
    cfg = cfg or load_config()
    hue = cfg['color']['hues'][color]
    quad, mask = find_zone_quad(img, hue, cfg)
    os.makedirs(os.path.join(LOG_DIR, 'calib'), exist_ok=True)
    cv2.imwrite(os.path.join(LOG_DIR, 'calib', 'floor_src.jpg'), img)
    if quad is None:
        cv2.imwrite(os.path.join(LOG_DIR, 'calib', 'floor_mask.jpg'), mask * 255)
        return ('%s 구역의 네 모서리를 못 찾았습니다. 구역 전체가 화면 안에 보이게 놓고 다시 (logs/calib/floor_mask.jpg 확인)' % color), None
    D = dist_cm
    h = ZONE_CM / 2.0
    # 바닥 좌표: x 앞, y 왼쪽. 가까운 왼쪽, 가까운 오른쪽, 먼 오른쪽, 먼 왼쪽
    floor_pts = [[D, h], [D, -h], [D + ZONE_CM, -h], [D + ZONE_CM, h]]
    calib = {'img_pts': quad.tolist(), 'floor_pts': floor_pts, 'color': color, 'dist_cm': D,
             'camera': cfg['camera'], 'at': time.strftime('%Y-%m-%d %H:%M:%S')}
    with open(CALIB_PATH, 'w', encoding='utf-8') as f:
        json.dump(calib, f, ensure_ascii=False, indent=1)

    # 확인 그림: 바닥 5cm 격자 + 차체 중심선(y=0)
    H = cv2.getPerspectiveTransform(quad, np.float32(floor_pts))
    Hinv = np.linalg.inv(H)
    vis = img.copy()
    for x in range(10, 60, 5):
        p = cv2.perspectiveTransform(np.float32([[[x, -15]], [[x, 15]]]), Hinv).reshape(-1, 2)
        cv2.line(vis, tuple(map(int, p[0])), tuple(map(int, p[1])), (0, 255, 255), 1)
        cv2.putText(vis, str(x), tuple(map(int, p[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 255), 1)
    for y in range(-15, 20, 5):
        p = cv2.perspectiveTransform(np.float32([[[10, y]], [[60, y]]]), Hinv).reshape(-1, 2)
        cv2.line(vis, tuple(map(int, p[0])), tuple(map(int, p[1])), (255, 0, 0) if y == 0 else (255, 255, 0), 2 if y == 0 else 1)
    cv2.polylines(vis, [quad.astype(np.int32)], True, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(LOG_DIR, 'calib', 'floor_check.jpg'), vis)
    h_img, w_img = img.shape[:2]
    bottom = cv2.perspectiveTransform(np.float32([[[w_img / 2.0, h_img - 1]]]), H).reshape(2)
    msg = ('저장: floor_calib.json | 화면 맨 아래 가운데 = 차체 중심에서 앞으로 %.1fcm (d = %.1fcm)\n'
           '확인: 파란 굵은 선(차체 중심선)이 검은 선 위에, 노란 격자가 색 구역 변과 맞아야 함') % (bottom[0], bottom[0] - 11.5)
    return msg, vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('color')
    ap.add_argument('dist_cm', type=float, help='차체 중심 -> 색 구역 가까운 변 (cm)')
    ap.add_argument('--image')
    a = ap.parse_args()
    cfg = load_config()
    if a.image:
        img = cv2.imread(a.image)
    else:
        set_camera_angle(cfg)
        cam = open_camera(cfg)
        try:
            time.sleep(1.5)
            img = cam.value.copy()
        finally:
            cam.stop()
    msg, vis = calibrate(img, a.color, a.dist_cm, cfg)
    print(msg)
    if vis is None:
        raise SystemExit(1)
    print('확인 그림: logs/calib/floor_check.jpg')


if __name__ == '__main__':
    main()
