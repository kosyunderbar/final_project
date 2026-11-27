# -*- coding: utf-8 -*-
"""화면 픽셀 <-> 바닥 좌표(cm). 원점 = 차체 중심, x = 앞, y = 왼쪽.

카메라가 몸통 앞에 고정이라 화면은 차체 중심을 보지 못한다(바닥이 보이기 시작하는 곳이 중심에서 약 15~21cm 앞).
그래서 '화면 속 선을 화면 가운데로'가 아니라, 선을 바닥 좌표로 옮겨 '차체 중심에서의 옆 오차 e'와
'선과 차의 방향 차이 ψ'로 조향한다 (2026-11-25 에이전트 회의: 제어 담당).

floor_calib.json (calib_floor.py로 만든다)이 없으면 config.json floor.default_*의 대략값을 쓴다 -> 실주행 전에 반드시 잰다.
"""
import json
import os

import cv2
import numpy as np

from common import HERE

CALIB_PATH = os.path.join(HERE, 'floor_calib.json')


class FloorMap(object):
    def __init__(self, cfg):
        f = cfg['floor']
        if os.path.exists(CALIB_PATH):
            with open(CALIB_PATH, encoding='utf-8') as fh:
                c = json.load(fh)
            img_pts, floor_pts = c['img_pts'], c['floor_pts']
            self.source = 'floor_calib.json (%s)' % c.get('at', '')
        else:
            img_pts, floor_pts = f['default_img_pts'], f['default_floor_pts']
            self.source = 'config 대략값 (calib_floor.py로 재야 함)'
        self.H = cv2.getPerspectiveTransform(np.float32(img_pts), np.float32(floor_pts))
        self.Hinv = np.linalg.inv(self.H)
        self.line_width_cm = f['line_width_cm']

    def to_floor(self, pts_px):
        p = np.float32(pts_px).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.H).reshape(-1, 2)

    def to_img(self, pts_cm):
        p = np.float32(pts_cm).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(p, self.Hinv).reshape(-1, 2)

    def expected_width_px(self, cx, row):
        """그 줄에서 선(폭 line_width_cm)이 몇 px로 보여야 하나"""
        x, y = self.to_floor([[cx, row]])[0]
        a, b = self.to_img([[x, y - self.line_width_cm / 2.0], [x, y + self.line_width_cm / 2.0]])
        return float(abs(a[0] - b[0]))


def fit_line(pts_cm):
    """바닥 점들에 y = a + b·x 를 맞춘다. 반환 (a, b) — a = 차체 중심(x=0)에서의 옆 오차, atan(b) = 방향 차이"""
    p = np.asarray(pts_cm, dtype=np.float64)
    if len(p) < 2 or np.ptp(p[:, 0]) < 1e-3:
        return None
    b, a = np.polyfit(p[:, 0], p[:, 1], 1)
    return float(a), float(b)
