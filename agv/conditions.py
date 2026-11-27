# -*- coding: utf-8 -*-
"""'언제 서나'와 '언제 출발하나'를 여기 한 곳에 모은다 (가이드 '바꿔 끼우는 자리').

    지금    정지 = 색을 보고 가운데까지 감      출발 = wait_s 초 지남
    나중    정지 = MQTT "태그 읽음" 도착        출발 = MQTT "작업 완료" 도착

구역별 표(rules)를 두어, 최종 계획(노랑만 색으로 서고 주황~빨강은 태그로 섬)에도 그대로 맞춘다.
MQTT를 붙일 때는 통신 쪽이 ctx.events 에 표시만 남기고, 아래 함수가 그 표시를 읽는다.
"""


class Conditions(object):
    def __init__(self, cfg):
        s = cfg['stop']
        self.cfg = cfg
        self.mode = s['mode']                      # 'color' | 'pass'
        self.wait_s = s['wait_s']
        self.extra_cm = s['extra_cm']
        # 구역별 정지 방법. 지금은 모두 'color'. 나중에 주황~빨강을 'tag'로 바꾼다.
        self.rules = {z: ('color' if z in s['stop_zones'] else 'pass') for z in cfg['route']['order']}
        if self.mode == 'pass':
            self.rules = {z: 'pass' for z in self.rules}

    def stops_at(self, zone):
        return self.rules.get(zone) != 'pass'

    def align_time(self, zone):
        """색이 아래 띠에서 사라진 뒤 더 갈 시간(초) = (4 + d)cm ÷ align_speed의 초속 (실측 모터 특성)"""
        from common import travel_time
        return travel_time(self.cfg, self.extra_cm.get(zone, 6), self.cfg['drive']['align_speed'])

    # ---- 정지 조건 -------------------------------------------------------
    def should_stop(self, ctx):
        rule = self.rules.get(ctx.target)
        if rule == 'color':
            return ctx.state == 'ALIGN' and (ctx.now - ctx.state_since) >= self.align_time(ctx.target)
        if rule == 'tag':   # 나중: 목적지 공정의 '태그 읽음'
            return ctx.events.pop('tag_read:' + ctx.target, False)
        return False

    # ---- 출발 조건 -------------------------------------------------------
    def arm(self, ctx):
        """정차에 들어갈 때 한 번 부른다"""
        ctx.armed_at = ctx.now
        ctx.events.pop('work_done:' + str(ctx.target), None)

    def should_depart(self, ctx):
        # 나중: return ctx.events.pop('work_done:' + ctx.target, False)
        return (ctx.now - ctx.armed_at) >= self.wait_s
