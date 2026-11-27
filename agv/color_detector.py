# -*- coding: utf-8 -*-
"""사진 -> 색 이름 또는 없음 (가이드 '색을 알아보는 방법')

1. 사진 아래 1/4, 가운데 60% 폭의 띠만 본다
2. HSV에서 채도·밝기가 충분한 점만 남긴다(S > 80, V > 60) -> 흰 바닥과 검은 선이 빠진다
3. 남은 점이 띠의 30%를 넘으면 그 점들의 색상(H) 대표값을 구한다
   (빨강은 0과 179 양쪽에 걸치므로 H를 원 위의 각도로 보고 평균 낸다)
4. 미리 재 둔 여섯 색 중 가장 가까운 색. 너무 멀면 없음
5. 같은 색이 confirm_frames장 이어져야 확정 (ColorConfirm)
"""
import math

import cv2
import numpy as np


def hue_dist(a, b):
    """OpenCV H(0~179)의 원 거리"""
    d = abs(a - b) % 180
    return min(d, 180 - d)


def circular_mean_h(hs):
    ang = hs.astype(np.float32) * (2.0 * math.pi / 180.0)
    m = math.atan2(float(np.mean(np.sin(ang))), float(np.mean(np.cos(ang))))
    return (m * 180.0 / (2.0 * math.pi)) % 180.0


class ColorReading(object):
    __slots__ = ('name', 'h', 's', 'v', 'fill', 'dist', 'weak')

    def __init__(self, name=None, h=None, s=None, v=None, fill=0.0, dist=None):
        self.name, self.h, self.s, self.v, self.fill, self.dist = name, h, s, v, fill, dist
        self.weak = None          # 채움이 min_fill보다 적어도 start_min_fill 이상이면 붙는 약한 색 이름 (출발 직후 판정용)


def band(img, c):
    hgt, wid = img.shape[:2]
    y0, y1 = int(hgt * c['band_top']), int(hgt * c['band_bottom'])
    half = int(wid * c['band_width'] / 2)
    x0, x1 = wid // 2 - half, wid // 2 + half
    return img[y0:y1, x0:x1], (x0, y0, x1, y1)


def measure(img, c):
    """띠 안의 색 점 비율과 H·S·V 대표값 (색 이름은 붙이지 않음) — calib_hsv에서도 쓴다"""
    roi, _ = band(img, c)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    sel = (hsv[..., 1] > c['min_s']) & (hsv[..., 2] > c['min_v'])
    fill = float(sel.mean())
    if not sel.any():
        return ColorReading(None, None, None, None, fill)
    px = hsv[sel]
    return ColorReading(None, circular_mean_h(px[:, 0]), float(np.median(px[:, 1])), float(np.median(px[:, 2])), fill)


class ColorDetector(object):
    def __init__(self, cfg):
        self.c = cfg['color']
        self.hues = dict(self.c['hues'])

    def detect(self, img):
        r = measure(img, self.c)
        wmin = self.c.get('start_min_fill', 0)
        if r.h is not None and wmin and r.fill >= wmin:
            wn, wd = min(((n, hue_dist(r.h, h)) for n, h in self.hues.items()), key=lambda t: t[1])
            if wd <= self.c['max_h_dist']:
                r.weak = wn
        if r.h is None or r.fill < self.c['min_fill']:
            return r
        name, dist = min(((n, hue_dist(r.h, h)) for n, h in self.hues.items()), key=lambda t: t[1])
        r.dist = dist
        if dist <= self.c['max_h_dist']:
            r.name = name
        return r


class ColorConfirm(object):
    """같은 색이 n장 이어져야 확정. 확정된 색이 없으면 None"""

    def __init__(self, n):
        self.n = n
        self.last = None
        self.count = 0

    def reset(self):
        self.last, self.count = None, 0

    def update(self, name):
        if name is not None and name == self.last:
            self.count += 1
        else:
            self.last, self.count = name, (1 if name is not None else 0)
        return self.last if (self.last is not None and self.count >= self.n) else None
