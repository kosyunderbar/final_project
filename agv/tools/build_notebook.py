# -*- coding: utf-8 -*-
"""run_agv.ipynb 생성기 (표준 라이브러리만): python3 tools/build_notebook.py  ->  ../run_agv.ipynb
노트북을 직접 고치지 말고 이 파일을 고친 뒤 다시 만든다."""
import json
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'run_agv.ipynb')

CELLS = [
('md', r'''
# AGV 주행 — 현장 노트북

이 노트북은 `agv/` 폴더 안에서 연다. 주행 방식은 **상황별 주행** 하나다(`situ.py`, 설계안_1126_상황별주행.md).

| 순서 | 이 화면에서 하는 일 |
|---|---|
| 0 | 수동 버튼(앞·뒤·좌·우·정지)이 듣는지, `사진 찍기`로 한 장 저장 |
| 준비 | 아래 **바닥 좌표 맞추기**(카메라 각도나 차체를 바꿨을 때만) |
| 1 | `서기 = 통과만` → `막 떠난 구역` 고르기 → speed 0.40 → `출발`. 차체 중심이 선 위로 가는지, 곁길(노랑 S자·초록-보라 혹)을 안 타는지 본다 |
| 2 | `달리며 사진 저장` 켜고 돌기 (아래 요약 칸에서 PASS 순서 확인) |
| 3 | `서기 = 색에서 서기`: 구역 가운데에 서고 5초 뒤 출발 |

**안전**: 위험하면 `정지`(또는 AGV를 들어 올림 → 선을 놓쳐 스스로 정지). 끝나면 마지막 칸 실행 + 커널 종료(카메라는 한 노트북만 쓸 수 있다).

**사진 더 모으기**: 문제가 생긴 자리에서 `장면 이름`을 적고 `사진 찍기` → `logs/snapshots/<장면>/`.
집에서 `python3 replay.py logs/snapshots/<장면> --no-safety`로 같은 장면을 다시 돌려 본다.
'''),
('code', r'''
# 준비: 설정, 카메라, 모터 (카메라 서보는 config.json의 camera.servo_* 값으로 맞춘다)
import json
import os
import threading
import time

import cv2
import ipywidgets as widgets
from IPython.display import display
from jetbot import Robot

import common
import main as agv
from fsm import DRIVE
from logger import NullLogger

cfg = common.load_config()
robot = Robot()
print('카메라 서보:', common.set_camera_angle(cfg))
camera = common.open_camera(cfg)
time.sleep(1.0)
print('카메라', camera.value.shape, '| 서기', cfg['stop']['mode'])
'''),
('code', r'''
# 화면 + 조작. 뒤에서 스레드 하나가 1초에 약 20번: (주행 중) Runner.step / (멈춤) 미리보기만
image = widgets.Image(format='jpeg', width=320, height=320)
state_label = widgets.Label(value='준비')
event_box = widgets.Textarea(value='', layout=widgets.Layout(width='440px', height='150px'))

mode_dd = widgets.Dropdown(options=[('통과만(색 이름)', 'pass'), ('색에서 서기', 'color')], value=cfg['stop']['mode'], description='서기')
start_dd = widgets.Dropdown(options=cfg['route']['order'], value=cfg['route']['start_after'], description='막 떠난 구역')
speed_s = widgets.FloatSlider(min=0.0, max=0.5, step=0.01, value=cfg['drive']['speed'], description='speed')
record_cb = widgets.Checkbox(value=False, description='달리며 사진 저장(5장마다)')

start_b = widgets.Button(description='출발', button_style='success')
stop_b = widgets.Button(description='정지', button_style='danger')
reset_b = widgets.Button(description='이상 해제', button_style='warning')
tag_txt = widgets.Text(value='yellow_cross', description='장면 이름')
snap_b = widgets.Button(description='사진 찍기', button_style='info')

lay = widgets.Layout(width='64px', height='40px')
mv = {k: widgets.Button(description=k, layout=lay) for k in ['앞', '뒤', '좌', '우', '■']}

S = {'alive': True, 'driving': False, 'runner': None, 'preview': None}
LOCK = threading.Lock()   # 뒤 스레드의 step과 출발/정지가 겹치지 않게 (닫힌 기록 파일에 쓰는 오류 방지)


def make_cfg():
    c = common.load_config()   # 출발할 때마다 config.json을 새로 읽는다 -> 파일을 고치면 커널 재시작 없이 적용
    c['camera'] = cfg['camera']
    c['stop']['mode'] = mode_dd.value
    c['route']['start_after'] = start_dd.value
    c['drive']['speed'] = speed_s.value
    c['loop']['save_every_n'] = 5 if record_cb.value else 0
    return c


def new_preview(*_):
    S['preview'] = agv.Runner(make_cfg(), camera, robot, logger=NullLogger())
    state_label.value = '미리보기'


def worker():
    period = 1.0 / cfg['loop']['hz']
    while S['alive']:
        t0 = time.time()
        try:
            with LOCK:
                driving = S['driving']
                r = S['runner'] if driving else S['preview']
                row = None
                if r is not None:
                    if driving:
                        r.speed[DRIVE] = speed_s.value
                        row = r.step()
                    else:
                        row = r.preview()
            if r is not None:
                img = agv.draw_overlay(r)
                if img is not None:
                    image.value = bytes(cv2.imencode('.jpg', img)[1])
                if row is not None and driving:
                    ctx = r.fsm.ctx
                    state_label.value = '%s | 목표 %s | lap %d | L %s R %s %s' % (
                        ctx.state, ctx.target, ctx.lap, row['left'], row['right'],
                        ('| ' + ctx.fault_reason) if ctx.fault_reason else '')
                    if row['event']:
                        event_box.value = '\n'.join(reversed(r.events))
        except Exception as e:   # 어떤 오류든 모터부터 세운다
            S['driving'] = False
            robot.stop()
            state_label.value = '오류: %r' % e
        dt = period - (time.time() - t0)
        time.sleep(dt if dt > 0 else 0.001)


def on_start(_):
    with LOCK:
        if S['driving']:
            return
        pv = S['preview']
        S['runner'] = agv.Runner(make_cfg(), camera, robot, log_prefix='run', tracker=pv.tracker if pv else None)
        S['driving'] = True
    event_box.value = '기록: ' + S['runner'].logger.path


def on_stop(_):
    with LOCK:
        S['driving'] = False
        if S['runner'] is not None:
            S['runner'].stop()
            S['runner'].logger.close()
        robot.stop()
    state_label.value = '정지'


def on_reset(_):
    if S['runner'] is not None:
        S['runner'].reset_fault()


def on_snap(_):
    d = os.path.join(common.LOG_DIR, 'snapshots', tag_txt.value.strip() or 'misc')
    os.makedirs(d, exist_ok=True)
    p = os.path.join(d, time.strftime('%H%M%S_') + '%03d.jpg' % int((time.time() % 1) * 1000))
    cv2.imwrite(p, camera.value)
    state_label.value = '저장 %s (%d장)' % (p, len(os.listdir(d)))


def manual(move, sec):
    if S['driving']:
        state_label.value = '주행 중에는 수동 버튼을 쓰지 않는다. 먼저 정지'
        return
    try:
        move()
        time.sleep(sec)
    finally:
        robot.stop()


mv['앞'].on_click(lambda b: manual(lambda: robot.forward(0.3), 0.3))
mv['뒤'].on_click(lambda b: manual(lambda: robot.backward(0.3), 0.3))
mv['좌'].on_click(lambda b: manual(lambda: robot.left(0.25), 0.2))
mv['우'].on_click(lambda b: manual(lambda: robot.right(0.25), 0.2))
mv['■'].on_click(lambda b: robot.stop())
start_b.on_click(on_start)
stop_b.on_click(on_stop)
reset_b.on_click(on_reset)
snap_b.on_click(on_snap)

new_preview()
th = threading.Thread(target=worker, daemon=True)
th.start()

pad = widgets.VBox([widgets.HBox([widgets.Label(layout=lay), mv['앞']]),
                    widgets.HBox([mv['좌'], mv['■'], mv['우']]),
                    widgets.HBox([widgets.Label(layout=lay), mv['뒤']])])
display(widgets.HBox([image, widgets.VBox([mode_dd, start_dd, speed_s, record_cb,
                                           widgets.HBox([start_b, stop_b, reset_b]),
                                           widgets.HBox([tag_txt, snap_b]), pad])]),
        state_label, event_box)
'''),
('md', r'''
## 바닥 좌표 맞추기 (처음 한 번, 10분)

`floor` 방식은 화면 속 선을 바닥 cm로 바꿔 **차체 중심**이 선 위에 오게 조향한다. 그러려면 화면 ↔ 바닥 관계를 한 번 재야 한다.

1. AGV를 **곧은 선 위 가운데**에 똑바로 놓는다. 앞에 색 구역 하나(주황이 좋다)가 **화면에 통째로** 보이게
2. 자로 **차체 중심(몸통 앞뒤 23cm의 가운데) → 색 구역의 가까운 변**까지 거리를 잰다
3. 아래 셀의 `COLOR`, `DIST_CM`만 바꾸고 실행 → 그림에서 파란 굵은 선이 검은 선 위에, 노란 격자가 구역 변과 맞으면 끝
'''),
('code', r'''
import calib_floor

COLOR, DIST_CM = 'orange', 24.5      # <- 여기만 바꾼다 (색, 차체 중심 -> 구역 가까운 변 cm)

msg, vis = calib_floor.calibrate(camera.value.copy(), COLOR, DIST_CM)
print(msg)
if vis is not None:
    display(widgets.Image(value=bytes(cv2.imencode('.jpg', vis)[1]), format='jpeg', width=320))
    new_preview()   # 새 바닥 좌표로 미리보기를 다시 만든다
'''),
('md', r'''
## 직진 속도 재기 (2분, 자 필요 없음. 배터리가 많이 바뀌었을 때)

모서리에서 '2cm 더 가기'를 시간으로 세므로 속도가 20% 넘게 틀리면 다시 잰다.
AGV를 **곧은 선 위**에 놓고, 앞 30~40cm에 색 구역이 보이게 한 뒤 실행 → 짧게 한 번·길게 한 번 간다 → 구역이 다가온 거리로 속도를 구해 저장한다.
'''),
('code', r'''
import importlib, calib_motion
importlib.reload(calib_motion)
if S['driving']:
    print('먼저 정지')
else:
    with LOCK:
        k = calib_motion.forward_speed(camera, robot, make_cfg())
    if k and 8 < k < 30:
        calib_motion.save_speed_unit(k)
        print('다음 출발부터 적용')
    else:
        print('저장 안 함 (값이 이상하거나 구역을 못 봄)')
'''),
('md', r'''
## 마지막 주행 요약

가장 최근 `logs/run_*.csv`에서 상태별 시간, 사건 순서(PASS/STOP/DEPART/SKIP/FAULT), 한 바퀴 일 시간을 본다.
'''),
('code', r'''
import collections
import csv
import glob

runs = sorted(glob.glob(os.path.join(common.LOG_DIR, 'run_*.csv')))
if not runs:
    print('주행 기록 없음')
else:
    rows = list(csv.DictReader(open(runs[-1])))
    print(runs[-1], len(rows), '줄')
    print('상태별 장수:', dict(collections.Counter(r['state'] for r in rows)))
    ms = sorted(float(r['loop_ms']) for r in rows if r['loop_ms'])
    if ms:
        print('한 바퀴 일: 중앙 %.1fms, 95%% %.1fms, 최대 %.1fms' % (ms[len(ms) // 2], ms[int(len(ms) * .95)], ms[-1]))
    print('선 찾음: %.1f%%' % (100.0 * sum(int(r['line_found']) for r in rows) / len(rows)))
    for r in rows:
        if r['event']:
            print('%7ss  lap %s  %s' % (r['t'], r['lap'], r['event']))
'''),
('md', r'''
## 끝내기

다른 노트북이나 `main.py`, `calib_hsv.py`에서 카메라를 쓰려면 아래 셀을 실행하고 **커널도 끈다**.
'''),
('code', r'''
S['driving'] = False
S['alive'] = False
time.sleep(0.3)
if S['runner'] is not None:
    S['runner'].logger.close()
robot.stop()
camera.stop()
print('정지 · 카메라 닫음')
'''),
]


def nb(cells):
    out = []
    for kind, src in cells:
        lines = [l + '\n' for l in src.strip('\n').split('\n')]
        lines[-1] = lines[-1].rstrip('\n')
        if kind == 'md':
            out.append({'cell_type': 'markdown', 'metadata': {}, 'source': lines})
        else:
            out.append({'cell_type': 'code', 'execution_count': None, 'metadata': {}, 'outputs': [], 'source': lines})
    return {'cells': out,
            'metadata': {'kernelspec': {'display_name': 'Python 3', 'language': 'python', 'name': 'python3'},
                         'language_info': {'name': 'python', 'version': '3.6.9'}},
            'nbformat': 4, 'nbformat_minor': 4}


if __name__ == '__main__':
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump(nb(CELLS), f, ensure_ascii=False, indent=1)
    print('wrote', os.path.normpath(OUT), len(CELLS), 'cells')
