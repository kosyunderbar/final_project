# -*- coding: utf-8 -*-
"""가짜 색 신호로 상태 흐름을 시험한다 (카메라·모터 없이): python3 tools/test_fsm.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from common import load_config          # noqa: E402
from conditions import Conditions       # noqa: E402
from fsm import FSM                     # noqa: E402

DT = 0.05
fails = []


def check(name, cond, detail=''):
    print('[%s] %s %s' % ('OK' if cond else 'NG', name, detail))
    if not cond:
        fails.append(name)


def feed(fsm, t, seq):
    """seq: [(색 또는 None, 장수), ...] -> 사건 목록"""
    evs = []
    for color, n in seq:
        for _ in range(n):
            ev = fsm.step(t[0], color)
            if ev:
                evs.append(ev)
            t[0] += DT
    return evs


cfg = load_config()
cfg['stop']['mode'] = 'color'
cfg['stop']['wait_s'] = 1.0
cond = Conditions(cfg)
f = FSM(cfg, cond)
t = [0.0]
f.start('yellow', 0.0)

# 노랑 위에서 출발: 노랑은 잠겨 있어 다시 서지 않는다
ev = feed(f, t, [('yellow', 10), (None, 12)])
check('출발한 노랑에 다시 서지 않음', not any(e.startswith('APPROACH') for e in ev), ev)
check('목표는 주황', f.ctx.target == 'orange')

# 순서가 아닌 색(두 칸 뒤 보라)은 기록만 (한 칸 뒤 초록은 따라잡기 대상이라 보라로 시험)
ev = feed(f, t, [('purple', 6), (None, 3)])
check('두 칸 뒤 색은 SKIP 한 번(따라잡기 안 함)', ev == ['SKIP:purple(target orange)'] and f.ctx.target == 'orange', ev)

# 주황: 확정 3장 -> APPROACH, 사라짐 -> ALIGN, 시간 -> STOP, 1초 -> DEPART
align_t = cond.align_time('orange')
ev = feed(f, t, [('orange', 6), (None, 2)])
check('주황 3장 연속 -> APPROACH, 사라짐 -> ALIGN', ev == ['APPROACH:orange', 'ALIGN:orange'], ev)
ev = feed(f, t, [(None, int(align_t / DT) + 2)])
check('더 가는 시간 뒤 STOP', ev == ['STOP:orange'] and f.ctx.state == 'STOP', (ev, align_t))
ev = feed(f, t, [(None, int(1.0 / DT) + 2)])
check('기다린 뒤 DEPART, 목표 초록', ev == ['DEPART:orange'] and f.ctx.target == 'green', ev)

# 출발 직후 색 무시(0.3초) + 떠난 색 잠금
ev = feed(f, t, [('orange', 8)])
check('떠난 주황은 다시 보지 않음', ev == [], ev)

# 보라 다음 파랑이 14cm 안에 와도 놓치지 않는지 (초록 -> 보라 통과 후 바로 파랑)
cfg2 = load_config()
cfg2['stop']['mode'] = 'pass'
f2 = FSM(cfg2, Conditions(cfg2))
t2 = [0.0]
f2.start('green', 0.0)
ev = feed(f2, t2, [(None, 8), ('purple', 5), (None, 2), ('blue', 5), (None, 3), ('red', 5)])
check('통과 모드: 보라 -> 파랑 -> 빨강 순서대로 PASS', ev == ['PASS:purple', 'PASS:blue', 'PASS:red'], ev)

# 한 바퀴: 빨강 다음은 노랑, lap 증가
ev = feed(f2, t2, [(None, 12), ('yellow', 5)])
check('빨강 다음 노랑 PASS, lap 1', ev == ['PASS:yellow'] and f2.ctx.lap == 1, (ev, f2.ctx.lap))

# ---- 11-27 색 놓침 복구 ----
def fresh(mode, start_after):
    cf = load_config()
    cf['stop']['mode'] = mode
    cf['stop']['wait_s'] = 1.0
    ff = FSM(cf, Conditions(cf))
    ff.start(start_after, 0.0)
    return ff


def feedw(fsm, t, seq):
    """seq: [(색, 약한 색, 장수)]"""
    evs = []
    for color, weak, n in seq:
        for _ in range(n):
            ev = fsm.step(t[0], color, weak)
            if ev:
                evs.extend(ev.split(','))
            t[0] += DT
    return evs


g = fresh('pass', 'orange'); tg = [0.0]
ev = feedw(g, tg, [(None, None, 10), ('purple', 'purple', 6), (None, None, 3)])
check('초록 놓친 뒤 보라 6장 -> 따라잡기', ev[-2:] == ['CATCHUP:green', 'PASS:purple'] and g.ctx.target == 'blue', ev)

g = fresh('color', 'orange'); tg = [0.0]
ev = feedw(g, tg, [(None, None, 10), ('purple', 'purple', 6), (None, None, 2)])
check('서기 모드: 초록 놓친 뒤 보라 -> 보라에 다가감', 'CATCHUP:green' in ev and 'APPROACH:purple' in ev and g.ctx.target == 'purple', ev)

g = fresh('pass', 'orange'); tg = [0.0]
ev = feedw(g, tg, [(None, None, 10), ('blue', 'blue', 8), (None, None, 3)])
check('두 칸 뒤 파랑은 SKIP만', ev == ['SKIP:blue(target green)'] and g.ctx.target == 'green', ev)

g = fresh('pass', 'blue'); tg = [0.0]
ev = feedw(g, tg, [(None, None, 10), ('yellow', 'yellow', 6), (None, None, 3)])
check('빨강 놓친 뒤 노랑 -> 따라잡기, lap 1', ev[-2:] == ['CATCHUP:red', 'PASS:yellow'] and g.ctx.lap == 1 and g.ctx.target == 'orange', (ev, g.ctx.lap))

g = fresh('pass', 'orange'); tg = [0.0]
ev = feedw(g, tg, [(None, None, 10), ('purple', 'purple', 3), (None, None, 3)])
check('보라 3장만이면 SKIP만', ev == ['SKIP:purple(target green)'] and g.ctx.target == 'green', ev)

g = fresh('pass', 'blue'); tg = [0.0]
ev = feedw(g, tg, [(None, 'red', 3), (None, None, 5)])
check('빨강 위에서 출발(약한 빨강 3장) -> PASS(start), 목표 노랑', ev == ['PASS:red(start)'] and g.ctx.target == 'yellow', ev)

g = fresh('pass', 'blue'); tg = [0.0]
ev = feedw(g, tg, [(None, 'blue', 4), (None, None, 5)])
check('떠난 색(파랑)이 약하게 보여도 변화 없음', ev == [] and g.ctx.target == 'red', ev)

g = fresh('pass', 'blue'); tg = [0.0]
ev = feedw(g, tg, [(None, None, 12), (None, 'red', 4), (None, None, 3)])
check('출발 0.5초 뒤의 약한 빨강은 무시', ev == [] and g.ctx.target == 'red', ev)

cf0 = load_config()
cf0['stop']['mode'] = 'pass'
cf0['color']['catchup_max'] = 0
cf0['color']['start_window_s'] = 0
g = FSM(cf0, Conditions(cf0)); g.start('orange', 0.0); tg = [0.0]
ev = feedw(g, tg, [(None, None, 10), ('purple', 'purple', 6), (None, None, 3)])
check('끄면 예전과 같음(SKIP만)', ev == ['SKIP:purple(target green)'] and g.ctx.target == 'green', ev)

# 이상 -> 해제
f.fault('시험', t[0])
check('FAULT 중에는 진행 안 함', f.step(t[0], 'green') == '' and f.ctx.state == 'FAULT')
f.reset(t[0])
check('reset -> DRIVE', f.ctx.state == 'DRIVE')

print('\n%d개 실패' % len(fails) if fails else '\n모두 통과')
sys.exit(1 if fails else 0)
