# -*- coding: utf-8 -*-
"""상황별 주행 (2026-11-26 회의, 사용자 제안: 직선 = 라인트레이싱 + 갈림길 규칙 + 모서리는 느리게 좌회전)

원칙: 선을 기억하지 않는다. 추측 항법으로 위치·방향을 세지 않는다. 지금 카메라에 보이는 것으로만 판단한다.
      회전은 시간·각도가 아니라 카메라가 끝낸다. 설계와 근거: 설계안_1126_상황별주행.md

상태
  LINE  직선(또는 살짝 굽은 선): 옆 오차 e와 방향 ψ로 3단계 조향. 많이 틀어지면 서서 제자리 회전으로 먼저 맞춘다
  APP   모서리 접근: 앞쪽 선이 왼쪽으로 휨(휨 각도 > bend_deg, 3장 연속). 느리게 직진
  COR   모서리: 맨 아랫줄(차체 13.6cm 앞)의 선 위치 yb 하나만 본다
          GO   직진. yb가 왼쪽 y_hi(7cm)에 닿거나 선이 왼쪽으로 사라지면 -> EXT
          EXT  extra_cm(2cm) 더 직진 -> TURN
          TURN 제자리 왼쪽 회전. yb가 y_lo(6cm) 안으로 돌아오면 -> GO
          나가는 곧은 선이 보이면: 차체가 그 선 위에 올 때까지(e ≥ -e_stop) 직진한 뒤 회전해 방향을 맞추고 LINE으로
  LOST  선을 놓침: 모서리가 올 구간이면 왼쪽으로(= COR의 TURN), 아니면 마지막으로 선이 있던 쪽으로 제자리 회전

모의 시험(tools/sim_situ.py)과 로봇이 이 파일의 같은 클래스를 쓴다.
"""
import math

import numpy as np


def fit_line(pts):
    p = np.asarray(pts, dtype=np.float64)
    if len(p) < 2 or np.ptp(p[:, 0]) < 1e-3:
        return None
    b, a = np.polyfit(p[:, 0], p[:, 1], 1)
    return float(a), float(b)


def straight_fit(fp, c):
    """곧은 선이 길게 보이면 (e cm, ψ rad), 아니면 None. e = 그 직선을 차체 중심(x=0)까지 늘린 옆 오차(+면 선이 왼쪽)"""
    if fp is None or len(fp) < c.get('straight_min_pts', 10):
        return None
    if float(fp[-1][0]) - float(fp[0][0]) < c.get('straight_min_len_cm', 8.0):
        return None
    ab = fit_line(fp)
    if ab is None:
        return None
    a, b = ab
    res = max(abs(float(p[1]) - (a + b * float(p[0]))) for p in fp)
    return (a, math.atan(b)) if res <= c.get('straight_res_cm', 0.35) else None


def bend_angle(fp):
    """휨 각도(도) = 추적 점 먼 절반의 방향 - 가까운 절반의 방향. +면 앞에서 왼쪽으로 휜다. 점이 모자라면 None
    (11-26 검증: 10° 넘게 3장 연속이면 네 모서리 모두 12~19cm 전에 켜지고, 직선·혹·노랑 뒤 갈림에서는 0번)"""
    if fp is None or len(fp) < 6:
        return None
    h = len(fp) // 2
    fn, ff = fit_line(fp[:max(h, 3)]), fit_line(fp[h:])
    if fn is None or ff is None:
        return None
    return math.degrees(math.atan(ff[1]) - math.atan(fn[1]))


class NearRow(object):
    """맨 아랫줄의 선 위치 yb(cm, +면 왼쪽) 전용 검출기.
    선 추적기(FloorLineTracker)는 화면 왼쪽 아래 귀퉁이에 걸린 선을 못 잡는다(그 자리 매트가 어두워 '양옆이 매트' 검사에서
    떨어짐 — 11-26 모서리 B 374~409줄 0/36장). 여기서는 그 검사를 빼는 대신
      - 직전 위치에서 이어지는 덩어리만 받고(jump_cm),
      - y_min보다 오른쪽 덩어리는 버린다 (로봇 그림자가 오른쪽 u 170~197, y -6.8cm에 계속 있다)."""

    def __init__(self, fm, rows, c):
        self.fm = fm
        self.rows = [int(r) for r in rows[:2]]         # 맨 아래 2줄 (13.6cm, 14.1cm 앞). 첫 줄에서 못 찾으면 둘째 줄
        self.min_w = c.get('near_min_w_px', 5)
        self.max_w = c.get('near_max_w_px', 110)       # 비스듬한 선은 가로로 넓게 잘린다
        self.edge_min_w = c.get('near_edge_min_w_px', 12)
        self.jump = c.get('near_jump_cm', 3.5)
        self.wide = c.get('near_fork_jump_cm', 9.0)      # 규칙 쪽으로 넘어갈 수 있는 거리
        # 'left'로 jump 넘게 건너뛸 때는 이 폭 이상인 덩어리만 (11-27 f·i: 화면 왼쪽 아래 u9~15의 폭 5~7칸 가짜 검정(y 8.0)으로 건너뛰어
        # 모서리를 일찍 시작하고 진짜 선(2cm)을 버려 6초 제자리 회전. 진짜 안쪽 갈래는 폭 15~20칸). 0이면 예전과 같음
        self.wide_min_w = c.get('near_fork_min_w_px', 0)

    def candidates(self, mask, r, with_w=False):
        """그 줄의 검은 덩어리들 -> [(y cm, 왼쪽 끝에 잘림?)] (with_w면 (y, 잘림, 폭 px))"""
        row = mask[r]
        w = row.shape[0]
        xs = np.flatnonzero(row)
        if xs.size == 0:
            return []
        br = np.flatnonzero(np.diff(xs) > 1)
        st = np.concatenate(([xs[0]], xs[br + 1]))
        en = np.concatenate((xs[br], [xs[-1]]))
        out = []
        for s, e in zip(st, en):
            wd = int(e - s + 1)
            if wd < self.min_w or wd > self.max_w or e >= w - 2:
                continue
            clipped = s <= 1
            if clipped and wd < self.edge_min_w:           # 화면 왼쪽 끝의 가짜 검정(u 0~17)
                continue
            y = float(self.fm.to_floor([[(s + e) / 2.0, r]])[0][1])
            out.append((y, clipped, wd) if with_w else (y, clipped))
        return out

    def find(self, mask, y_prev, y_min, prefer=None):
        """(yb, 왼쪽 끝에 잘림?) 또는 (None, False). y_prev = 직전 위치(없으면 None), y_min = 이보다 오른쪽은 버림.
        prefer: 직전 위치에서 이어지는 덩어리가 둘이면(갈림길) 'left' = 왼쪽 것, 'right' = 오른쪽 것, None = 가까운 것
        (11-26 사진: 빨강->노랑 모서리 끝 갈림에서 본길 +7~8cm, 곁길 +5~6cm가 같이 보인다 -> 구간 규칙 'left'로 고른다)"""
        if mask is None:
            return None, False
        for r in self.rows:
            cw = [cd for cd in self.candidates(mask, r, with_w=True) if cd[0] >= y_min]
            if prefer == 'left' and y_prev is not None and self.wide_min_w:
                cw = [cd for cd in cw if cd[0] <= y_prev + self.jump or cd[2] >= self.wide_min_w]
            cands = [(cd[0], cd[1]) for cd in cw]
            if not cands:
                continue
            if y_prev is None:
                return min(cands) if prefer == 'right' else max(cands)
            # 구간 규칙이 'left'면 왼쪽으로는 멀리 있는 선으로도 넘어간다('right'면 오른쪽으로). 반대쪽으로는 jump까지만.
            # (11-26 오후: 빨강->노랑 모서리 끝 갈림에서 안쪽(왼쪽) 갈래가 3.5cm보다 멀리 보여 바깥 갈래를 계속 탔다)
            if prefer == 'left':
                near = [cd for cd in cands if y_prev - self.jump <= cd[0] <= y_prev + self.wide]
            elif prefer == 'right':
                near = [cd for cd in cands if y_prev - self.wide <= cd[0] <= y_prev + self.jump]
            else:
                near = [cd for cd in cands if abs(cd[0] - y_prev) <= self.jump]
            if not near:
                continue
            if prefer == 'left':
                return max(near)
            if prefer == 'right':
                return min(near)
            return min(near, key=lambda cd: abs(cd[0] - y_prev))
        return None, False


class Corner(object):
    """모서리 하나를 도는 판단 (순수 계산 — 모의 시험과 로봇이 같이 쓴다)."""

    def __init__(self, c, v_cm_s):
        self.y_hi = c.get('cor_y_hi_cm', 7.0)
        self.y_lo = c.get('cor_y_lo_cm', 6.0)
        self.y_lo_first = c.get('cor_first_y_lo_cm', self.y_lo)   # 첫 회전만 더 많이 돈다(11-27 사용자 제안). 없으면 예전과 같음
        self.extra_s = c.get('cor_extra_cm', 2.0) / max(v_cm_s, 0.1)
        self.e_stop = c.get('cor_e_stop_cm', 0.5)
        self.exit_psi = c.get('cor_exit_psi_deg', 5.0)
        # 나가는 곧은 선이 아랫줄 검출기가 따라가던 선과 같은 선이어야 믿는 구간 규칙 (11-27 에이전트 회의: 빨강->노랑 끝에서
        # 곧은 바깥 갈래(S자 곁길)를 '나가는 선'으로 보고 모서리를 끝냈다. 같은 선이면 차이 0~0.9cm, 바깥 갈래면 2.2~2.5cm)
        self.same_rules = c.get('cor_same_line_rules', [])
        self.same_cm = c.get('cor_same_line_cm', 1.5)
        self.same_e_max = c.get('cor_same_line_e_max_cm', 3.0)
        self.row_x = c.get('cor_bottom_row_x_cm', 13.6)
        # 갈림길 장면 (11-27 둘째 팀 회의): 곧은 선(|ψ| < fork_psi)이 아랫줄을 지나는 자리가 yb보다 fork_gap 넘게 오른쪽이고
        # yb ≥ fork_yb_min이면 = 곧은 바깥 갈래(S자 곁길)와 휜 안쪽 본길이 같이 보임. 이걸 보면 6cm에서 멈추지 않고
        # 본길이 fork_y_lo까지 올 때까지 돌고 모서리를 끝낸다 (6cm에서 멈추면 그 방향이 곁길과 나란해 곁길을 따라갔다)
        self.fork_rules = c.get('cor_fork_rules', [])
        self.fork_gap = c.get('cor_fork_gap_cm', 2.0)
        self.fork_psi = c.get('cor_fork_psi_deg', 12.0)
        self.fork_yb_min = c.get('cor_fork_yb_min_cm', 4.5)
        self.fork_y_lo = c.get('cor_fork_y_lo_cm', 2.0)
        self.fork_max_s = c.get('cor_fork_max_s', 2.5)
        self.fork_since = None
        self.fork_off = False
        self.phase = 'EXT'
        self.t_ext = 0.0
        self.npiv = 0
        self.lasty = None
        self.turn_since = None

    def start(self, now, lasty, turn_now=False):
        """turn_now: 선을 이미 놓친 채 들어옴(= 이미 모서리 안) -> 더 가지 않고 바로 왼쪽으로"""
        self.lasty = lasty
        self.npiv = 0
        if turn_now:
            self._turn(now)
        else:
            self.phase, self.t_ext = 'EXT', now + self.extra_s

    def _turn(self, now):
        self.phase, self.npiv, self.turn_since = 'TURN', self.npiv + 1, now

    def step(self, now, yb, clipped, st, rule=None, near=None):
        """yb: 아랫줄 선 위치(없으면 None), clipped: 그 덩어리가 화면 왼쪽 끝에 잘림, st: 곧은 선 (e, ψ) 또는 None.
        반환 (phase, 끝났나)"""
        if yb is not None and clipped:
            yb = max(yb, self.y_hi)                        # 잘린 덩어리의 가운데는 실제보다 안쪽 -> '아직 왼쪽 끝'으로 본다
        # 나가는 곧은 선: 한 번이라도 돈 뒤에만 믿는다(그 전에 보이는 곧은 선은 들어온 쪽 직선)
        # near = 가까운 쪽(18cm 안) 직선 (e, ψ). 곁길은 위쪽에서 다시 휘어 '길게 곧은 선'(st)은 안 나온다 (11-27 h 45.7초)
        if (rule in self.fork_rules and self.fork_since is None and not self.fork_off and near is not None and yb is not None
                and self.npiv >= 1 and abs(math.degrees(near[1])) < self.fork_psi and yb >= self.fork_yb_min
                and yb - (near[0] + self.row_x * math.tan(near[1])) > self.fork_gap):
            self.fork_since = now                          # 갈림길 장면을 봤다 -> 이 모서리는 본길이 가운데 올 때까지 돈다
        if self.fork_since is not None and now - self.fork_since > self.fork_max_s:
            self.fork_since, self.fork_off = None, True    # 너무 오래 걸리면 예전 동작으로
        fork = self.fork_since is not None
        if fork and self.phase != 'TURN':
            self._turn(now)
        stv = st is not None and self.npiv >= 1 and abs(math.degrees(st[1])) < 50.0
        if stv and rule in self.same_rules and yb is not None:
            y_at_row = st[0] + self.row_x * math.tan(st[1])     # 곧은 선이 아랫줄을 지나는 자리
            if abs(y_at_row - yb) > self.same_cm or st[0] > self.same_e_max:
                stv = False                                    # 다른 갈래 -> 끝내지 않고 아랫줄 선으로 계속
        done = False
        if self.phase == 'GO':
            if stv:
                if st[0] >= -self.e_stop:                  # 차체가 나가는 선 위에 거의 옴 -> 돌아서 방향 맞추기
                    self._turn(now)
            elif (yb is None and (self.lasty is None or self.lasty > 0)) or (yb is not None and yb >= self.y_hi):
                self.phase, self.t_ext = 'EXT', now + self.extra_s
        elif self.phase == 'EXT':
            if now >= self.t_ext:
                self._turn(now)
        else:                                              # TURN
            if fork:
                if yb is not None and yb <= self.fork_y_lo:
                    done = True                            # 본길이 몸 앞 가운데로 옴 -> 남은 방향은 직선 주행이 맞춘다
            elif stv and st[0] >= -1.0:
                if abs(math.degrees(st[1])) <= self.exit_psi:
                    done = True
            elif yb is not None and yb <= (self.y_lo_first if self.npiv == 1 else self.y_lo):
                self.phase = 'GO'
        if yb is not None:
            self.lasty = yb
        return self.phase, done


class SituDriver(object):
    """한 걸음마다 (왼쪽, 오른쪽, 상태 이름, 이상 사유 또는 None)을 돌려준다."""

    def __init__(self, cfg, cm_per_s):
        self.cfg = cfg
        self.c = cfg['ctrl']
        c = self.c
        self.pm = c['pivot_motor']
        self.v_cor = c.get('cor_speed', 0.30)
        self.v_cor_cm_s = cm_per_s(cfg, self.v_cor)
        self.bend_deg = c.get('bend_deg', 10.0)
        self.bend_frames = c.get('bend_frames', 3)
        self.Ls = c.get('line_e_len_cm', 12.0)
        self.reset()

    def reset(self):
        self.state = 'LINE'
        self.bend_run = 0
        self.align = 0.0               # 제자리 회전으로 방향 맞추는 중이면 방향(+1 왼쪽 / -1 오른쪽)
        self.align_run = 0
        self.cor = None
        self.app_since = 0.0
        self.lasty = None
        self.last_side = 1.0
        self.lost_since = None
        self.last_cmd = (0.0, 0.0)
        self.corner_started = False    # 이번 걸음에 모서리(APP)로 들어갔나 — main이 '구간당 한 번'을 표시
        self.corner_finished = False   # 이번 걸음에 모서리를 끝냈나
        self.reset_tracker = False     # main이 선 추적기를 처음 상태로 되돌려야 하나 (갈림길 회전으로 모서리를 끝냈을 때)
        self.s_deg = 0.0
        self.yb = None
        self.yb_at = -1e9              # 아랫줄 선을 마지막으로 본 시각
        self.last_psi = 0.0            # 마지막에 본 선의 방향(°)
        self.last_y0 = 0.0             # 마지막에 본 선의 가장 가까운 점 옆 위치(cm)
        self.target = None             # 지금 목표 구역 (main이 걸음마다 넣는다. 핸들 방식을 켤 모서리 고르기용)
        self.steer_cor = None          # 핸들 방식 판단을 한 모서리(Corner 객체)
        self.steer_ok = False          # 이 모서리에서 핸들 방식을 쓰나 (모서리를 시작할 때의 목표로 정함)
        self.steer_latch = False       # 나가는 선을 한 번 봤으면 이 모서리에서는 예전 동작으로
        self.steer_since = None

    # ---- 직선 -----------------------------------------------------------------
    def _line_cmd(self, e, psi, v, rule, allow_pivot=True):
        c = self.c
        psi_deg = math.degrees(psi)
        s = psi_deg + math.degrees(e / self.Ls)            # 방향 오차 + 옆 오차를 각도로 (12cm 앞의 선을 겨누는 셈)
        self.s_deg = s
        if rule == 'straight':                             # 노랑 X자: 곧게 지나간다. 튀는 사진은 무시
            if abs(psi_deg) > c.get('x_max_psi_deg', 15.0) or abs(e) > c.get('x_max_e_cm', 3.0):
                self.align = 0.0
                return v, v, 'LINE_X'
            allow_pivot = False
        if allow_pivot:
            if self.align:
                if abs(s) <= c.get('line_align_done_deg', 3.0) or (s > 0) != (self.align > 0):
                    self.align = 0.0
                else:
                    return -self.align * self.pm, self.align * self.pm, 'ALIGN'
            if abs(s) > c.get('line_align_deg', 10.0):
                self.align_run += 1
                if self.align_run >= 2:                    # 한 장만 튄 것으로는 서지 않는다
                    self.align = 1.0 if s > 0 else -1.0
                    return -self.align * self.pm, self.align * self.pm, 'ALIGN'
            else:
                self.align_run = 0
        else:
            self.align = 0.0
        dead = c.get('line_dead_deg', 2.0)
        if abs(s) <= dead:
            return v, v, 'LINE'
        t0, t1 = c.get('line_turn_min', 0.10), c.get('line_turn_max', 0.15)   # 0.05 이하는 실제로 안 돈다
        turn = t0 + (t1 - t0) * min(1.0, (abs(s) - dead) / 8.0)
        turn = turn if s > 0 else -turn
        return v - turn, v + turn, 'LINE'

    # ---- 한 걸음 ----------------------------------------------------------------
    def step(self, now, found, fp, e, psi, yb_fn, rule, corner_ok, v):
        """_step_core(판단·명령) 뒤에, 켜져 있으면 모서리 안의 모터 값만 '핸들 방식'으로 바꾼다.
        (11-27 에이전트 팀: 모서리 판단(Corner)은 한 글자도 안 바꾼다. 서서 도는 대신 앞으로 가며 돌고, 곧게 가는 대신 살짝 휜다)"""
        l, r, mode, fault = self._step_core(now, found, fp, e, psi, yb_fn, rule, corner_ok, v)
        c = self.c
        cor = self.cor if self.state == 'COR' else None
        if cor is None or fault or not c.get('cor_steer', False):
            self.steer_cor = None
            return l, r, mode, fault
        if cor is not self.steer_cor:                       # 새 모서리: 시작할 때의 목표로 켤지 정한다
            self.steer_cor, self.steer_latch, self.steer_since = cor, False, None
            self.steer_ok = self.target in c.get('cor_steer_targets', [])
        if not self.steer_ok or cor.npiv < 1 or self.steer_latch:
            return l, r, mode, fault
        st = straight_fit(fp, c) if found else None
        stv = st is not None and abs(math.degrees(st[1])) < 50.0
        if stv and rule in cor.same_rules and self.yb is not None:
            y_at = st[0] + cor.row_x * math.tan(st[1])
            if abs(y_at - self.yb) > cor.same_cm or st[0] > cor.same_e_max:
                stv = False
        if stv:                                             # 나가는 곧은 선이 보이면 마무리는 예전 동작(방향 맞추기)으로
            self.steer_latch = True
            return l, r, mode, fault
        if self.steer_since is None:
            self.steer_since = now
        if now - self.steer_since > c.get('cor_steer_max_s', 6.0):
            return l, r, mode, fault
        if mode == 'COR_TURN':
            return self._out(c.get('cor_steer_tight', [-0.20, 0.60])[0], c.get('cor_steer_tight', [-0.20, 0.60])[1], 'COR_STEER_T')
        if mode in ('COR_GO', 'COR_EXT'):
            return self._out(c.get('cor_steer_wide', [0.20, 0.50])[0], c.get('cor_steer_wide', [0.20, 0.50])[1], 'COR_STEER_W')
        return l, r, mode, fault

    def _step_core(self, now, found, fp, e, psi, yb_fn, rule, corner_ok, v):
        """found/fp/e/psi: 선 추적 결과(fp = 바닥 점, e·psi = 가까운 쪽 직선). yb_fn(y_prev, y_min, prefer) -> (yb, clipped)
        corner_ok: 지금 구간에서 모서리를 시작해도 되나. v: 직선 속도(모터 값)"""
        c = self.c
        self.corner_started = self.corner_finished = False
        st = straight_fit(fp, c) if found else None
        if found:
            self.lost_since = None
            self.last_side = 1.0 if float(fp[0][1]) >= 0 else -1.0
            self.last_psi = math.degrees(psi)
            self.last_y0 = float(fp[0][1])

        if self.state == 'COR':
            yb, clipped = yb_fn(self.cor.lasty, c.get('near_y_min_cm', -1.0), rule if rule in ('left', 'right') else None)
            if yb is not None:
                self.yb_at = now
            elif self.cor.lasty is not None and now - self.yb_at <= c.get('near_hold_s', 0.3):
                yb = self.cor.lasty                         # 색 구역 위에서 몇 장 놓치는 것은 직전 값으로 버틴다
            self.yb = yb
            phase, done = self.cor.step(now, yb, clipped, st, rule, (e, psi) if found else None)
            if done:
                # 갈림길 회전으로 끝났으면 선 추적기를 처음 상태로 (11-27 넷째 팀 회의: 071743에서 추적기가 모서리 도중 곁길에 붙은 채
                # 끝나 곁길을 따라감. 처음 상태 추적기는 'left'로 다시 찾아 끝난 직후 첫 장부터 본길을 잡았다)
                self.reset_tracker = self.cor.fork_since is not None and c.get('cor_fork_reset_tracker', False)
                self.state, self.cor, self.align = 'LINE', None, 0.0
                self.corner_finished = True
                return self._out(v, v, 'COR_DONE')
            if phase == 'TURN':
                if now - self.cor.turn_since > c.get('cor_turn_max_s', 6.0):
                    return self._out(0.0, 0.0, 'COR_TURN', '모서리에서 %.0f초 넘게 돌았는데 선이 안 들어옴' % c.get('cor_turn_max_s', 6.0))
                return self._out(-self.pm, self.pm, 'COR_TURN')
            return self._out(self.v_cor, self.v_cor, 'COR_' + phase)

        if self.state == 'APP':
            y_prev = float(fp[0][1]) if found else self.lasty
            yb, clipped = yb_fn(y_prev, c.get('near_y_min_app_cm', -4.0), rule if rule in ('left', 'right') else None)
            if yb is None and found:
                yb = float(fp[0][1])
            self.yb = yb
            hit = yb is not None and (yb >= c.get('cor_y_hi_cm', 7.0) or clipped)
            gone = yb is None and not found and self.lasty is not None and self.lasty > 3.0
            if hit or gone:
                self.state = 'COR'
                self.cor = Corner(c, self.v_cor_cm_s)
                self.cor.start(now, yb if yb is not None else self.lasty)
                return self._out(self.v_cor, self.v_cor, 'COR_EXT')
            if yb is not None:
                self.lasty = yb
            if now - self.app_since > c.get('app_max_s', 8.0):         # 잘못 켜진 것 -> 직선으로 되돌림
                self.state = 'LINE'
            if found and abs(math.degrees(psi)) < 12.0:                 # 가까운 쪽이 곧을 때만 살짝 조향(제자리 회전 없음)
                l, r, _ = self._line_cmd(e, psi, self.v_cor, rule, allow_pivot=False)
                return self._out(l, r, 'APP')
            return self._out(self.v_cor, self.v_cor, 'APP')

        # ---- LINE ----
        if found:
            d = bend_angle(fp)
            self.bend_run = self.bend_run + 1 if (corner_ok and d is not None and d > self.bend_deg) else 0
            if self.bend_run >= self.bend_frames:
                self.state, self.bend_run, self.app_since, self.align = 'APP', 0, now, 0.0
                self.lasty = float(fp[0][1])
                self.corner_started = True
                return self._out(self.v_cor, self.v_cor, 'APP')
            if st is not None:
                l, r, m = self._line_cmd(st[0], st[1], v, rule)
            else:                                           # 곧지 않은 선: 가까운 쪽(18cm 안) 직선으로
                l, r, m = self._line_cmd(e, psi, v, rule)
            return self._out(l, r, m)

        # 선이 안 보임
        if self.lost_since is None:
            self.lost_since = now
        blind = now - self.lost_since
        if blind <= c.get('lost_hold_s', 0.3):                         # 색 구역 위에서 3장쯤 놓칠 수 있다 -> 직전 동작 유지
            return self._out(self.last_cmd[0], self.last_cmd[1], 'HOLD')
        if corner_ok and self.last_psi >= c.get('lost_corner_psi_deg', 15.0):   # 마지막에 본 선이 왼쪽으로 꺾여 있었을 때만 모서리로 본다
            self.state = 'COR'
            self.cor = Corner(c, self.v_cor_cm_s)
            self.cor.start(now, 8.0, turn_now=True)
            self.corner_started = True
            return self._out(-self.pm, self.pm, 'COR_TURN')
        if blind > c.get('lost_max_s', 2.5):
            return self._out(0.0, 0.0, 'LOST', '선을 %.1f초 넘게 못 찾음' % c.get('lost_max_s', 2.5))
        if abs(self.last_psi) < c.get('lost_straight_psi_deg', 10.0) and abs(self.last_y0) < c.get('lost_straight_y_cm', 3.0):
            return self._out(self.v_cor, self.v_cor, 'LOST_GO')
        s = self.last_side
        return self._out(-s * self.pm, s * self.pm, 'LOST')

    def _out(self, left, right, mode, fault=None):
        self.last_cmd = (left, right)
        return left, right, mode, fault
