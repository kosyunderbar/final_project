# -*- coding: utf-8 -*-
"""AGV 주행 — 한 걸음의 일 (1초에 약 20번):

    사진 받기 -> 색 판정 -> 상태 결정 -> 선 찾기 -> 상황별 주행(situ.py) -> 모터 -> 기록

주행 방식은 하나뿐이다: **상황별 주행** (2026-11-26, 설계안_1126_상황별주행.md)
    직선 = 카메라가 본 선으로 차체 중심 맞추기 / 갈림길 = 구간 규칙 / 모서리 = 느리게 직진 + 제자리 좌회전, 카메라가 회전을 끝냄

터미널(JetBot 컨테이너 안, 카메라를 다른 노트북이 잡고 있지 않을 때):
    python3 main.py                  # config.json 그대로
    python3 main.py --mode pass      # 서지 않고 달리며 색 이름만 출력
    python3 main.py --speed 0.40 --seconds 120
  멈출 때: Ctrl+C (안 되면 AGV 전원 스위치)

노트북에서: run_agv.ipynb (Runner를 스레드로 돌리고 화면에 상태를 그린다)
"""
import argparse
import math
import threading
import time

import cv2

from color_detector import ColorDetector, hue_dist
from common import LOG_DIR, cm_per_s, load_config, open_camera, set_camera_angle, travel_time
from conditions import Conditions
from drive import Drive
from fsm import ALIGN, APPROACH, DRIVE, FAULT, FSM, MOVING, STOP
from line_tracker import FloorLineTracker
from logger import RunLogger
from situ import NearRow, SituDriver

STATE_BGR = {DRIVE: (0, 200, 0), APPROACH: (0, 200, 255), ALIGN: (0, 140, 255), STOP: (255, 120, 0), FAULT: (0, 0, 255)}


class Runner(object):
    """camera.value(BGR) 와 robot.left_motor/right_motor 만 있으면 돈다 (시험에서는 가짜를 넣는다)"""

    def __init__(self, cfg, camera, robot, clock=time.time, log_prefix='run', logger=None, tracker=None):
        self.cfg = cfg
        self.camera = camera
        self.drive = Drive(robot, cfg)
        self.tracker = tracker if tracker is not None else FloorLineTracker(cfg)   # 노트북에서 출발할 때마다 다시 만들지 않게
        self.tracker.reset()
        self.colors = ColorDetector(cfg)
        self.cond = Conditions(cfg)
        self.fsm = FSM(cfg, self.cond)
        self.clock = clock
        self.logger = logger or RunLogger(LOG_DIR, log_prefix, cfg['loop']['save_every_n'])
        d = cfg['drive']
        self.speed = {DRIVE: d['speed'], APPROACH: d['approach_speed'], ALIGN: d['align_speed']}
        self.safety = cfg['safety']
        self.ctrl = cfg['ctrl']
        self.stop_event = threading.Event()
        self.last_frame = None
        self.last_frame_at = None
        self.last = {}               # 화면 표시용 마지막 결과
        self.events = []             # 최근 사건
        self.fsm.start(cfg['route']['start_after'], clock())
        self.started_at = clock()
        # 상황별 주행 (situ.py): 직선 = 카메라 직접 / 모서리 = 느리게 좌회전, 카메라가 회전을 끝냄
        self.situ = SituDriver(cfg, cm_per_s)
        self.near = NearRow(self.tracker.fm, self.tracker.rows, self.ctrl)
        self.mode = 'IDLE'
        self.rule = 'none'
        self.pending_fault = None
        self.corner_target = None        # 이 목표 구간에서 이미 모서리를 시작했나 (구간당 한 번)
        self.corner_done_at = -1e9
        self.yellow_until = 0.0
        self.after_yellow = False        # 노랑을 본 뒤 ~ 주황을 지날 때까지
        self.yellow_first = -1e9         # 이번 노랑을 처음 본 시각
        self.yellow_last = -1e9          # 노랑을 마지막으로 본 시각

    # ---- 구간 규칙 -------------------------------------------------------------
    def _cm_time(self, cm, motor):
        """모터 값 motor로 cm만큼 가는 시간 (실측 모터 특성)"""
        return travel_time(self.cfg, cm, motor)

    def _branch_rule(self, now, col):
        b = self.cfg['branch']
        ctx = self.fsm.ctx
        yh = self.colors.hues.get('yellow')
        if col.name == 'yellow' or (col.h is not None and col.fill > 0.05 and yh is not None and hue_dist(col.h, yh) <= 6):
            self.yellow_until = now + b['straight_after_yellow_s']   # 노랑이 조금이라도 보이면 바로 직진 (X자 교차)
            self.after_yellow = True
            if now - self.yellow_last > 1.0:
                self.yellow_first = now                              # 새로 만난 노랑
            self.yellow_last = now
        # '직진'은 노랑을 처음 본 뒤 straight_max_s까지만 (11-26: 노랑이 5%만 보여도 계속 늘어나 6초 동안 켜져 있었다)
        if now < self.yellow_until and now - self.yellow_first < b.get('straight_max_s', 4.0):
            return 'straight'
        # 노랑을 본 뒤로는 주황을 지날 때까지 '오른쪽'으로 고정한다(X자 뒤 갈림길: 오른쪽이 본길, 왼쪽 호가 곁길).
        # 목표 구역으로만 정하면 '색에서 서기'에서 노랑 출발 전까지 목표가 노랑이라 '왼쪽'(= 곁길)으로 돌아간다 (11-26 회의)
        if self.after_yellow:
            if ctx.target in ('yellow', 'orange'):
                return 'right'
            self.after_yellow = False
        r = b['rules'].get(ctx.target, 'none')
        if r == 'left' and now - ctx.last_zone_at > b['left_max_s']:
            return 'none'
        return r

    def _corner_allowed(self, now):
        """모서리를 시작해도 되나: 맵 순서상 모서리가 올 구간에서만, 구간마다 한 번만, 노랑 X자 근처가 아닐 때, 앞 모서리 직후 lockout_cm 지난 뒤"""
        c = self.ctrl
        tgt = self.fsm.ctx.target
        if not c['corner_enabled'] or self.rule == 'straight':
            return False
        if tgt not in c['corner_targets'] or self.corner_target == tgt:
            return False
        if now - self.yellow_last < c.get('corner_after_yellow_s', 1.0):   # 노랑 X자가 아직 화면 근처 -> 모서리 판정 금지
            return False
        return now - self.corner_done_at > self._cm_time(c['lockout_cm'], self.cfg['drive']['speed'])

    def _situ_drive(self, now, line, v):
        """상황별 주행 (situ.py). 선 기억·추측 항법 없음"""
        d = self.situ
        fp = getattr(line, 'floor_pts', None) if line.found else None
        mask = getattr(self.tracker, 'last_mask', None)

        def yb_fn(y_prev, y_min, prefer=None):
            return self.near.find(mask, y_prev, y_min, prefer) if self.near is not None else (None, False)
        d.target = self.fsm.ctx.target                    # 핸들 방식을 켤 모서리 고르기용 (situ.cor_steer_targets)
        left, right, self.mode, fault = d.step(now, line.found and fp is not None and len(fp) > 0, fp, line.e_cm, line.psi,
                                               yb_fn, self.rule, self._corner_allowed(now), v)
        if d.corner_started:
            self.corner_target = self.fsm.ctx.target      # 이 구간의 모서리는 시작함 (구간당 한 번). 도는 중에 목표가 바뀌어도 끝까지 돈다
        if d.corner_finished:
            self.corner_done_at = now
        if getattr(d, 'reset_tracker', False):
            self.tracker.reset()                          # 갈림길 회전으로 끝난 모서리: 곁길에 붙은 추적 기억을 버린다
            d.reset_tracker = False
        if fault:
            self.pending_fault = fault
        return self.drive.set(left, right)

    # ---- 한 걸음의 일 ------------------------------------------------------------
    def step(self):
        t0 = self.clock()
        now = t0
        ctx = self.fsm.ctx

        # 1. 사진 받기 (새 사진이 안 오면 정지)
        frame = self.camera.value
        if frame is not self.last_frame:
            self.last_frame, self.last_frame_at = frame, now
        elif self.last_frame_at is not None and now - self.last_frame_at > self.safety['frame_timeout_s']:
            self._fault('사진이 %.1f초 동안 새로 안 옴' % self.safety['frame_timeout_s'], now)
        if frame is None:
            self.drive.stop()
            return None

        # 2. 색 판정
        col = self.colors.detect(frame)

        # 3. 상태 결정
        ev = self.fsm.step(now, col.name, getattr(col, 'weak', None))

        # 4. 선 찾기 (갈림길이면 구간 규칙으로 고른다)
        prev_rule = self.rule
        self.rule = self._branch_rule(now, col)
        # 노랑 X자를 벗어나 '직진' -> '오른쪽'이 되는 순간 선 추적기를 처음부터 다시 찾게 한다 (11-27 다섯째 팀 회의: 074121에서
        # X자 안에서 추적기 기억이 왼쪽 호(곁길)에 붙은 채 규칙이 바뀌어 곁길을 탔다. 다시 찾으면 'right'로 본길을 고른다).
        # 노랑이 사라져서 끝났을 때만 (straight_max_s 제한으로 끝나면 아직 X자 안이라 대각선을 고를 수 있다)
        if (prev_rule == 'straight' and self.rule == 'right' and now >= self.yellow_until
                and self.cfg['branch'].get('reset_tracker_after_x', False)):
            self.tracker.reset()
        line = self.tracker.track(frame, self.rule)
        if ctx.state in MOVING and now - ctx.last_zone_at > self.safety['no_color_max_s']:
            ev = self._fault('색 없이 %.0f초 넘게 달림' % self.safety['no_color_max_s'], now) or ev

        # 5. 모터 (선 놓침·모서리에서 너무 오래 돎은 상황별 주행이 스스로 판단해 pending_fault로 알린다)
        state = ctx.state
        if state in MOVING:
            left, right = self._situ_drive(now, line, self.speed[state])
            if self.pending_fault:
                reason, self.pending_fault = self.pending_fault, None
                ev = self._fault(reason, now) or ev
                left = right = 0.0
        else:
            self.drive.stop()
            self.mode = 'IDLE'
            left = right = 0.0

        loop_ms = (self.clock() - t0) * 1000.0
        if loop_ms > self.safety['loop_timeout_s'] * 1000.0 and now - self.started_at > 1.0:   # 출발 직후 1초는 봐준다
            self.drive.stop()
            ev = self._fault('한 걸음이 %.0fms (제한 %.0fms)' % (loop_ms, self.safety['loop_timeout_s'] * 1000), now) or ev

        # 6. 기록 (yb = 아랫줄 선 위치 cm, s_deg = 직선 조향 오차 °, cor = 모서리 단계/회전 횟수)
        if ev:
            self.events.append('%.1f %s' % (now, ev))
            self.events = self.events[-8:]
        d = self.situ
        row = {'loop_ms': '%.1f' % loop_ms, 'state': ctx.state, 'lap': ctx.lap, 'zone_idx': ctx.zone_idx,
               'target': ctx.target, 'seen': col.name or '', 'confirmed': self.fsm.confirm.last or '',
               'h': '' if col.h is None else '%.1f' % col.h, 's': '' if col.s is None else '%.0f' % col.s,
               'v': '' if col.v is None else '%.0f' % col.v, 'fill': '%.2f' % col.fill,
               'line_found': int(line.found), 'e_cm': '%.2f' % line.e_cm, 'psi_deg': '%.1f' % math.degrees(line.psi),
               'mode': self.mode, 'rule': self.rule,
               'yb': '' if d.yb is None else '%.1f' % d.yb, 's_deg': '%.1f' % d.s_deg,
               'cor': '' if d.cor is None else '%s/%d' % (d.cor.phase, d.cor.npiv),
               'left': '%.3f' % left, 'right': '%.3f' % right, 'event': ev}
        self.logger.log(row, frame)
        self.last = {'frame': frame, 'line': line, 'color': col, 'row': row}
        return row

    def preview(self):
        """모터를 움직이지 않고 색 판정·선 추적만 해서 화면용 결과를 남긴다 (출발 전 확인, 수동 조작 중)"""
        frame = self.camera.value
        if frame is None:
            return None
        col = self.colors.detect(frame)
        self.rule = self._branch_rule(self.clock(), col)
        line = self.tracker.track(frame, self.rule)
        row = {'left': '0.000', 'right': '0.000', 'state': 'PREVIEW', 'seen': col.name or '',
               'line_found': int(line.found), 'event': '',
               'e_cm': '%.2f' % line.e_cm, 'psi_deg': '%.1f' % math.degrees(line.psi), 'mode': 'PREVIEW', 'rule': self.rule}
        self.last = {'frame': frame, 'line': line, 'color': col, 'row': row}
        return row

    def _fault(self, reason, now):
        self.drive.stop()
        return self.fsm.fault(reason, now)

    # ---- 반복 ----------------------------------------------------------------
    def run(self, seconds=None, on_step=None):
        period = 1.0 / self.cfg['loop']['hz']
        t_end = None if seconds is None else time.time() + seconds
        try:
            while not self.stop_event.is_set():
                t0 = time.time()
                row = self.step()
                if on_step is not None and row is not None:
                    on_step(self, row)
                if t_end is not None and time.time() > t_end:
                    break
                dt = period - (time.time() - t0)
                if dt > 0:
                    time.sleep(dt)
        finally:
            self.drive.stop()
            self.logger.close()

    def stop(self):
        self.stop_event.set()
        self.drive.stop()

    def reset_fault(self, start_after=None):
        self.fsm.reset(self.clock(), start_after)
        self.tracker.reset()
        self.situ.reset()
        self.corner_target = None


def draw_overlay(runner):
    """화면 표시용: 선 위치, 색 띠, 상태, 목표 구역"""
    last = runner.last
    if not last:
        return None
    img = last['frame'].copy()
    h, w = img.shape[:2]
    c = runner.cfg['color']
    y0, y1 = int(h * c['band_top']), int(h * c['band_bottom']) - 1
    half = int(w * c['band_width'] / 2)
    cv2.rectangle(img, (w // 2 - half, y0), (w // 2 + half, y1), (255, 255, 255), 1)
    lc = runner.cfg['line']
    for frac in (lc['band_top'], lc['band_bottom']):
        cv2.line(img, (0, int(h * frac)), (w - 1, int(h * frac)), (200, 200, 200), 1)
    line = last['line']
    if line.found and getattr(line, 'pts', None):
        pts = [(int(x), int(y)) for x, y in line.pts]
        for p, q in zip(pts, pts[1:]):
            cv2.line(img, p, q, (0, 255, 0), 2)
        for p in pts:
            cv2.circle(img, p, 2, (0, 0, 255), -1)
    elif line.found and line.cx is not None:
        cv2.circle(img, (int(line.cx), int(line.cy if line.cy is not None else h * 0.8)), 6, (0, 255, 0), 2)
        cv2.line(img, (w // 2, h - 1), (int(line.cx), int(line.cy if line.cy is not None else h * 0.8)), (255, 0, 0), 2)
    ctx = runner.fsm.ctx
    col = last['color']
    state = last['row'].get('state', ctx.state)
    color = STATE_BGR.get(state, (200, 200, 200))
    row = last['row']
    lines = ['%s  lap %d  %s' % (state, ctx.lap, row.get('mode', '')),
             'target %s  rule %s' % (ctx.target, row.get('rule', '')),
             'e %scm psi %s' % (row.get('e_cm', '-'), row.get('psi_deg', '-')),
             'see %s %s' % (col.name or '-', '' if col.h is None else 'H%.0f %.0f%%' % (col.h, col.fill * 100)),
             'L %s R %s' % (last['row']['left'], last['row']['right'])]
    if ctx.state == FAULT:
        lines.append('FAULT')
    for i, t in enumerate(lines):
        cv2.putText(img, t, (4, 14 + 14 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, t, (4, 14 + 14 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['color', 'pass'])
    ap.add_argument('--speed', type=float)
    ap.add_argument('--seconds', type=float)
    ap.add_argument('--start-after', help='출발할 때 막 떠난 구역 색 (기본 config route.start_after)')
    ap.add_argument('--save-every', type=int, help='n장마다 사진 저장 (0 = 저장 안 함)')
    a = ap.parse_args()
    cfg = load_config()
    if a.mode:
        cfg['stop']['mode'] = a.mode
    if a.speed is not None:
        cfg['drive']['speed'] = a.speed
    if a.start_after:
        cfg['route']['start_after'] = a.start_after
    if a.save_every is not None:
        cfg['loop']['save_every_n'] = a.save_every

    from jetbot import Robot
    robot = Robot()
    set_camera_angle(cfg)
    camera = open_camera(cfg)
    runner = None
    try:
        time.sleep(1.0)   # 카메라 첫 사진
        runner = Runner(cfg, camera, robot)
        print('기록:', runner.logger.path, '| 멈추려면 Ctrl+C')

        def show(r, row):
            if row['event']:
                print('%6.1fs  %-8s %s' % (r.clock() - r.logger.t0, row['state'], row['event']))
        runner.run(a.seconds, show)
    except KeyboardInterrupt:
        print('\nCtrl+C: 정지')
    finally:
        if runner is not None:
            runner.stop()
        robot.left_motor.value = 0.0
        robot.right_motor.value = 0.0
        robot.stop()
        camera.stop()


if __name__ == '__main__':
    main()
