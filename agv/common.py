# -*- coding: utf-8 -*-
"""설정 읽기와 하드웨어 공통 함수 (Python 3.6, JetBot 컨테이너)

jetbot / SCSCtrl 은 함수 안에서만 불러온다 -> 하드웨어가 없는 곳(노트북 PC, AGV 본체, 시험)에서도 import 된다.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, 'config.json')
HSV_CALIB_PATH = os.path.join(HERE, 'hsv_calib.json')
LOG_DIR = os.path.join(HERE, 'logs')

# SCSCtrl(서보)은 pip 패키지가 아니라 JETANK 폴더 안에 있다. pyserial은 sample/.agv_deps에 사본이 있다.
EXTRA_PATHS = ['/workspace/JETANK', '/home/jetbot/JETANK',
               '/workspace/sample/.agv_deps', '/home/jetbot/sample/.agv_deps']


def _strip_notes(d):
    if isinstance(d, dict):
        return {k: _strip_notes(v) for k, v in d.items() if not k.startswith('_')}
    return d


def load_config(path=CONFIG_PATH):
    """config.json + (있으면) hsv_calib.json 의 색 값을 합친다"""
    with open(path, encoding='utf-8') as f:
        cfg = _strip_notes(json.load(f))
    if os.path.exists(HSV_CALIB_PATH):
        with open(HSV_CALIB_PATH, encoding='utf-8') as f:
            calib = json.load(f)
        cfg['color']['hues'].update(calib.get('hues', {}))
        cfg['color']['calib'] = calib
    return cfg


def add_hw_paths():
    for d in EXTRA_PATHS:
        if os.path.isdir(d) and d not in sys.path:
            sys.path.append(d)


def set_camera_angle(cfg):
    cam = cfg['camera']
    if not cam.get('servo_enabled'):
        return False
    add_hw_paths()
    from SCSCtrl import TTLServo
    TTLServo.servoAngleCtrl(cam['servo_id'], cam['servo_angle'], 1, 100)
    return True


def open_camera(cfg):
    from jetbot import Camera
    return Camera.instance(width=cfg['camera']['width'], height=cfg['camera']['height'])


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ---- 모터 특성 (2026-11-25 실측: 3초 직진 0.10 -> 2cm 미만, 0.20 -> 7.5cm, 0.30 -> 12.5cm) ----------
#   초속(cm) = (모터 값 - 데드밴드) × 기울기,  데드밴드 0.05, 기울기 16.7 cm/s
def cm_per_s(cfg, motor):
    d = cfg['drive']
    return max(0.0, (motor - d['motor_deadband']) * d['cm_s_per_unit'])


def motor_for(cfg, cm_s):
    """원하는 초속(cm)을 내는 모터 값. 0 이하면 0"""
    d = cfg['drive']
    return 0.0 if cm_s <= 0 else d['motor_deadband'] + cm_s / d['cm_s_per_unit']


def travel_time(cfg, cm, motor):
    v = cm_per_s(cfg, motor)
    return max(0.0, cm) / v if v > 0.05 else 0.0
