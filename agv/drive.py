# -*- coding: utf-8 -*-
"""모터 명령을 한 곳에서. 속도 제한과 정지는 여기서만 한다."""
from common import clamp


class Drive(object):
    """robot: jetbot.Robot (또는 시험용 가짜). left_motor.value / right_motor.value 를 쓴다
    (PJT08 주행에서 실물로 확인한 방식. robot.set_motors 는 교안에 없다)."""

    def __init__(self, robot, cfg):
        self.robot = robot
        d = cfg['drive']
        self.max_motor = d['max_motor']
        self.min_motor = d['min_motor']
        self.left = 0.0
        self.right = 0.0

    def set(self, left, right):
        self.left = clamp(left, self.min_motor, self.max_motor)
        self.right = clamp(right, self.min_motor, self.max_motor)
        self.robot.left_motor.value = self.left
        self.robot.right_motor.value = self.right
        return self.left, self.right

    def stop(self):
        self.left = self.right = 0.0
        try:
            self.robot.left_motor.value = 0.0
            self.robot.right_motor.value = 0.0
        finally:
            try:
                self.robot.stop()
            except Exception:
                pass
