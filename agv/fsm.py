# -*- coding: utf-8 -*-
"""상태와 넘어가는 규칙 (가이드 '상태')

    DRIVE    선을 따라가며 색을 찾는다          -> 다음 차례 색이 confirm_frames장 연속: APPROACH (pass 구역이면 통과 기록)
    APPROACH 속도를 줄여 계속 간다              -> 그 색이 아래 띠에서 사라짐: ALIGN
    ALIGN    정해진 거리만큼 더 간다            -> conditions.should_stop: STOP
    STOP     모터 0, 출발 조건을 기다린다        -> conditions.should_depart: DRIVE (다음 구역으로)
    FAULT    모터 0, 사람이 볼 때까지 그대로     -> reset()

방금 떠난 색을 다시 보지 않는 규칙:
  - 출발할 때 그 색을 잠그고, 띠에 아무 색도 없는 사진이 unlock_empty_frames장 이어지면 푼다
  - 출발 직후 ignore_after_depart_s 동안은 모든 색을 무시한다 (보라-파랑 사이가 14cm뿐이라 짧게)
  - 순서에서 다음 차례가 아닌 색은 기록만 하고 서지 않는다
"""
from color_detector import ColorConfirm

DRIVE, APPROACH, ALIGN, STOP, FAULT = 'DRIVE', 'APPROACH', 'ALIGN', 'STOP', 'FAULT'
MOVING = (DRIVE, APPROACH, ALIGN)


class Ctx(object):
    """조건 함수에 넘기는 '지금 상황'"""

    def __init__(self):
        self.now = 0.0
        self.state = DRIVE
        self.state_since = 0.0
        self.target = None          # 다음에 설 구역 색
        self.zone_idx = 0           # route.order 안의 위치
        self.lap = 0
        self.armed_at = 0.0
        self.departed_at = 0.0
        self.last_zone_at = 0.0     # 마지막으로 구역을 지나거나 떠난 시각 (색 없이 너무 오래 달림 판정)
        self.locked = None          # 방금 떠난 색
        self.empty_run = 0
        self.gone_run = 0
        self.fault_reason = ''
        self.events = {}            # 나중: MQTT 쪽이 남기는 표시 {'tag_read:orange': True, ...}


class FSM(object):
    def __init__(self, cfg, conditions):
        self.cfg = cfg
        self.cond = conditions
        self.order = cfg['route']['order']
        c = cfg['color']
        self.confirm = ColorConfirm(c['confirm_frames'])
        self.unlock_n = c['unlock_empty_frames']
        self.ignore_s = c['ignore_after_depart_s']
        # 11-27 팀 회의(색 놓침): 출발 직후 다음 색이 약하게 보이면 지나온 것으로 / 목표를 놓쳤는데 그다음 색이 확정되면 따라잡기
        self.start_window = c.get('start_window_s', 0.0)
        self.start_frames = c.get('start_frames', 2)
        self.catchup_max = c.get('catchup_max', 0)
        self.catchup_frames = c.get('catchup_frames', 5)
        self._start_run = 0
        self.started_at = 0.0
        self.ctx = Ctx()
        self._last_skip = None
        self.start(cfg['route']['start_after'], 0.0)

    # ---- 바깥에서 부르는 것 ---------------------------------------------
    def start(self, start_after, now):
        """start_after 구역에 서 있다가 막 출발한 것으로 시작한다"""
        x = self.ctx
        i = self.order.index(start_after)
        x.zone_idx = (i + 1) % len(self.order)
        x.target = self.order[x.zone_idx]
        x.locked = start_after
        x.empty_run = 0
        x.departed_at = x.last_zone_at = now
        self.started_at = now
        self._start_run = 0
        self._go(DRIVE, now)
        self.confirm.reset()

    def fault(self, reason, now):
        if self.ctx.state != FAULT:
            self.ctx.fault_reason = reason
            self._go(FAULT, now)
            return 'FAULT:' + reason
        return None

    def reset(self, now, start_after=None):
        """이상에서 사람이 다시 시작. 지금 목표 구역을 그대로 이어 간다(start_after를 주면 그 구역부터)"""
        self.ctx.fault_reason = ''
        if start_after:
            self.start(start_after, now)
        else:
            self.ctx.last_zone_at = now
            self.confirm.reset()
            self._go(DRIVE, now)

    def step(self, now, color_name, weak=None):
        """색 판정 결과 하나로 상태를 한 칸 진행한다. 반환: 사건 문자열(없으면 '')
        weak: 채움이 모자라도 붙는 약한 색 이름 (출발 직후 start_window_s 안에서만 쓴다)"""
        x = self.ctx
        x.now = now
        if x.state == FAULT:
            return ''

        # 출발 직후: 다음 차례 색이 화면 아래에 조금이라도 start_frames장 보이면 그 구역 위에서 출발한 것 -> 지나온 것으로
        # (11-27 064900: 빨강 끝에 놓여 빨강이 채움 0.16으로 0.2초만 보였고, 출발 직후 0.3초 색 무시에 걸려 놓침 -> 목표가 빨강에 묶임)
        if self.start_window and x.state == DRIVE and now - self.started_at <= self.start_window:
            if weak is not None and weak == x.target:
                self._start_run += 1
                if self._start_run >= self.start_frames:
                    tgt = x.target
                    self._advance(now, leave=tgt, departed=False)
                    return 'PASS:%s(start)' % tgt
            else:
                self._start_run = 0

        # 방금 떠난 색 잠금 풀기
        if color_name is None:
            x.empty_run += 1
            if x.locked and x.empty_run >= self.unlock_n:
                x.locked = None
        else:
            x.empty_run = 0
        seen = color_name
        if now - x.departed_at < self.ignore_s or (seen is not None and seen == x.locked):
            seen = None
        confirmed = self.confirm.update(seen)

        ev = ''
        if x.state == DRIVE:
            if confirmed == x.target:
                if self.cond.stops_at(x.target):
                    self._go(APPROACH, now)
                    x.gone_run = 0
                    ev = 'APPROACH:' + x.target
                else:
                    ev = 'PASS:' + x.target
                    self._advance(now, leave=x.target, departed=False)   # 통과는 색 잠금만, 무시 시간 없음
            elif confirmed is not None and confirmed != x.target and self._catchup_ok(confirmed):
                # 목표를 놓쳤는데 그다음 색이 확실히 보임 -> 놓친 구역은 지나온 것으로 하고 따라잡는다 (11-27 024316: 초록 채움 0.28로 놓친 뒤
                # 보라·파랑·빨강을 다 SKIP해 빨강->노랑 갈림 규칙이 틀림). catchup_max칸까지만 건너뛴다
                missed = x.target
                self._advance(now, leave=missed, departed=False)
                ev = 'CATCHUP:%s' % missed
                if self.cond.stops_at(x.target):
                    self._go(APPROACH, now)
                    x.gone_run = 0
                    ev += ',APPROACH:' + x.target
                else:
                    ev += ',PASS:' + x.target
                    self._advance(now, leave=x.target, departed=False)
            elif confirmed is not None and confirmed != x.target and confirmed != self._last_skip:
                ev = 'SKIP:%s(target %s)' % (confirmed, x.target)   # 다음 차례가 아닌 색: 기록만, 서지 않음
            self._last_skip = confirmed
        elif x.state == APPROACH:
            x.gone_run = x.gone_run + 1 if color_name != x.target else 0
            if x.gone_run >= 2:
                self._go(ALIGN, now)
                ev = 'ALIGN:' + x.target
        elif x.state == ALIGN:
            if self.cond.should_stop(x):
                self._go(STOP, now)
                self.cond.arm(x)
                ev = 'STOP:' + x.target
        elif x.state == STOP:
            if self.cond.should_depart(x):
                ev = 'DEPART:' + x.target
                self._advance(now, leave=x.target)
                self._go(DRIVE, now)
        return ev

    # ---- 안에서만 ----------------------------------------------------------
    def _catchup_ok(self, confirmed):
        if not self.catchup_max or confirmed not in self.order:
            return False
        k = (self.order.index(confirmed) - self.ctx.zone_idx) % len(self.order)
        return 1 <= k <= self.catchup_max and self.confirm.count >= self.catchup_frames

    def _go(self, state, now):
        self.ctx.state = state
        self.ctx.state_since = now

    def _advance(self, now, leave, departed=True):
        """departed=True: 정차했다 출발 (ignore_after_depart_s 동안 색 무시) / False: 서지 않고 통과"""
        x = self.ctx
        x.locked = leave
        x.empty_run = 0
        x.last_zone_at = now
        if departed:
            x.departed_at = now
        x.zone_idx = (x.zone_idx + 1) % len(self.order)
        if x.zone_idx == 0:
            x.lap += 1
        x.target = self.order[x.zone_idx]
        self.confirm.reset()
