# -*- coding: utf-8 -*-
"""주행 CSV 기록 (가이드의 열 그대로) + 사진 저장"""
import csv
import os
import time

import cv2

FIELDS = ['t', 'loop_ms', 'state', 'lap', 'zone_idx', 'target', 'seen', 'confirmed',
          'h', 's', 'v', 'fill', 'line_found', 'e_cm', 'psi_deg', 'mode', 'rule',
          'yb', 's_deg', 'cor', 'left', 'right', 'event']


class NullLogger(object):
    """기록하지 않는 자리채움 (미리보기용)"""
    path = ''
    t0 = time.time()

    def log(self, row, frame=None):
        pass

    def close(self):
        pass


class RunLogger(object):
    def __init__(self, log_dir, prefix='run', save_every_n=0):
        stamp = time.strftime('%Y%m%d_%H%M%S')
        if not os.path.isdir(log_dir):
            os.makedirs(log_dir)
        self.path = os.path.join(log_dir, '%s_%s.csv' % (prefix, stamp))
        self.frame_dir = os.path.join(log_dir, 'frames_%s' % stamp)
        self.save_every_n = save_every_n
        self.f = open(self.path, 'w', newline='', encoding='utf-8')   # 컨테이너 기본 인코딩이 ASCII라 한글 사건에서 오류가 났다
        self.w = csv.DictWriter(self.f, fieldnames=FIELDS, extrasaction='ignore')
        self.w.writeheader()
        self.n = 0
        self.t0 = time.time()

    def log(self, row, frame=None):
        row = dict(row)
        row.setdefault('t', '%.3f' % (time.time() - self.t0))
        self.w.writerow(row)
        self.n += 1
        if self.n % 20 == 0:
            self.f.flush()
        if frame is not None and self.save_every_n and self.n % self.save_every_n == 0:
            if not os.path.isdir(self.frame_dir):
                os.makedirs(self.frame_dir)
            cv2.imwrite(os.path.join(self.frame_dir, '%06d_%s.jpg' % (self.n, row.get('state', ''))), frame)

    def close(self):
        if not self.f.closed:
            self.f.close()
