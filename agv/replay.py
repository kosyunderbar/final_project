# -*- coding: utf-8 -*-
"""저장한 사진으로 베이스라인을 돌려 본다 (카메라·모터 없이, AGV 본체나 PC에서)

    python3 replay.py <사진 폴더> [--out logs/replay] [--mode pass] [--every 1] [--limit 0]

- 사진을 찍은 순서(파일 수정 시각)대로 카메라 대신 넣는다. 한 장 = 1/hz 초로 친다
- 모터는 가짜. 색 판정·상태·선 추적·기록이 모두 실제 코드 그대로 돈다
- 결과: <out>/run_*.csv, <out>/sheet_XX.jpg(화면 표시 48장 묶음), 요약 출력
- 온라인 주에 집에서 '한 바퀴 연속 사진'으로 색 판정과 상태 흐름을 시험하는 용도 (가이드 마지막 절)
"""
import argparse
import collections
import glob
import os

import cv2
import numpy as np

from common import load_config
from logger import RunLogger
from main import Runner, draw_overlay


class FakeMotor(object):
    value = 0.0


class FakeRobot(object):
    def __init__(self):
        self.left_motor, self.right_motor = FakeMotor(), FakeMotor()

    def stop(self):
        self.left_motor.value = self.right_motor.value = 0.0


class FileCamera(object):
    def __init__(self, files, size):
        self.files, self.size, self.i, self.value = files, size, -1, None

    def next(self):
        self.i += 1
        if self.i >= len(self.files):
            return False
        img = cv2.imread(self.files[self.i])
        if img.shape[1::-1] != self.size:
            img = cv2.resize(img, self.size, interpolation=cv2.INTER_AREA)
        self.value = img
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('folder')
    ap.add_argument('--out', default=os.path.join('logs', 'replay'))
    ap.add_argument('--mode', choices=['color', 'pass'], default='pass')
    ap.add_argument('--every', type=int, default=1, help='시트에 n장마다 하나씩')
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--start-after', help='출발할 때 막 떠난 구역 색 (기본 config route.start_after)')
    ap.add_argument('--dt', type=float, default=0.0, help='사진 한 장 사이 시간(초). 0이면 1/hz (달리며 5장마다 저장했으면 0.25)')
    ap.add_argument('--no-safety', action='store_true', help='시간 제한 이상 판정을 끈다(띄엄띄엄 찍은 사진용)')
    a = ap.parse_args()

    cfg = load_config()
    cfg['stop']['mode'] = a.mode
    if a.start_after:
        cfg['route']['start_after'] = a.start_after
    if a.no_safety:
        cfg['safety']['no_color_max_s'] = 1e9
        cfg['ctrl']['lost_max_s'] = 1e9
        cfg['ctrl']['cor_turn_max_s'] = 1e9
    files = sorted(glob.glob(os.path.join(a.folder, '*.jpg')), key=os.path.getmtime)
    if a.limit:
        files = files[:a.limit]
    cam = FileCamera(files, (cfg['camera']['width'], cfg['camera']['height']))
    sim = {'t': 0.0}
    period = a.dt if a.dt else 1.0 / cfg['loop']['hz']
    os.makedirs(a.out, exist_ok=True)
    cam.next()
    r = Runner(cfg, cam, FakeRobot(), clock=lambda: sim['t'], logger=RunLogger(a.out, 'replay'))

    seen = collections.Counter()
    found = 0
    events = []
    thumbs = []
    n = 0
    while True:
        row = r.step()
        n += 1
        seen[row['seen'] or '-'] += 1
        found += row['line_found']
        if row['event']:
            events.append('%4d %s %s' % (cam.i, os.path.basename(files[cam.i])[:22], row['event']))
        if r.fsm.ctx.state == 'FAULT':
            r.reset_fault()
        if (cam.i % a.every) == 0:
            t = cv2.resize(draw_overlay(r), (192, 192))
            cv2.putText(t, str(cam.i), (150, 186), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
            thumbs.append(t)
        sim['t'] += period
        if not cam.next():
            break
    r.logger.close()

    per = 48
    for s in range(0, len(thumbs), per):
        chunk = thumbs[s:s + per]
        while len(chunk) < per:
            chunk.append(np.zeros_like(thumbs[0]))
        grid = np.vstack([np.hstack(chunk[i * 8:(i + 1) * 8]) for i in range(6)])
        cv2.imwrite(os.path.join(a.out, 'sheet_%02d.jpg' % (s // per)), grid, [cv2.IMWRITE_JPEG_QUALITY, 85])

    print('사진 %d장 | 선 찾음 %d (%.1f%%)' % (n, found, 100.0 * found / max(1, n)))
    print('색 판정(장):', dict(seen))
    print('사건 %d개 (앞 40개):' % len(events))
    for e in events[:40]:
        print('  ' + e)
    print('기록:', r.logger.path)


if __name__ == '__main__':
    main()
