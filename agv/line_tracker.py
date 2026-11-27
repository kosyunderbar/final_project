# -*- coding: utf-8 -*-
"""사진 -> 검은 선의 위치 (바닥 좌표 cm).

    tracker = FloorLineTracker(cfg)
    r = tracker.track(frame_bgr, rule)   # r.found, r.floor_pts(차체 좌표 cm), r.e_cm, r.psi, r.pts(화면 px)
"""
import math
import os
import sys

import cv2
import numpy as np


class LineResult(object):
    """found / offset·direction(-1~1, 옛 방식·화면 표시용) / cx,cy(화면 표시 점)
    e_cm: 차체 중심에서 선까지 옆 오차(cm, +면 선이 왼쪽) / psi: 선과 차의 방향 차이(rad, +면 선이 왼쪽으로 향함)
    bend: 앞에서 왼쪽으로 꺾이는 모서리가 보임 / bend_x: 차체 중심에서 꺾이기 시작하는 곳까지(cm) / pts: 고른 점(화면 px)"""
    __slots__ = ('found', 'offset', 'direction', 'cx', 'cy', 'info', 'e_cm', 'psi', 'psi_far', 'bend', 'bend_x', 'pts', 'floor_pts')

    def __init__(self, found, offset=0.0, direction=0.0, cx=None, cy=None, info=None):
        self.found = found
        self.offset = offset
        self.direction = direction
        self.cx = cx
        self.cy = cy
        self.info = info or {}
        self.e_cm = 0.0
        self.psi = 0.0
        self.psi_far = 0.0
        self.bend = False
        self.floor_pts = None
        self.bend_x = None
        self.pts = []


def dark_line_mask(img, cfg_line):
    """검은 선 마스크. PJT08 자동 라벨에서 실측한 기준:
    주변 매트보다 (1 - dark_ratio) 이상 어둡고, 회색 dark_max_gray 미만, 채도 dark_max_sat 미만.
    보라(채도 90~125)·파랑(140~200) 영역은 어두워도 채도로 빠진다."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    sat = hsv[..., 1]
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    local = cv2.dilate(blur, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41))).astype(np.float32)
    dark = (gray < cfg_line['dark_ratio'] * local) & (gray < cfg_line['dark_max_gray']) & (sat < cfg_line['dark_max_sat'])
    bh_min = cfg_line.get('blackhat_min', 0)
    if bh_min:
        # 블랙햇: 가로로 메운 밝기 - 원래 밝기 -> 주변보다 어두운 가는 띠만 남는다.
        # 색 구역 위 선은 반사로 밝게 찍혀(회색 85~120, 주황 바탕 110~125) 비율 기준에 안 걸렸다. 블랙햇은 선 22~91, 바탕 13 이하 (11-25 실측)
        k = cfg_line.get('blackhat_kernel', 61)
        bh = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (k, 1)))
        dark |= (bh >= bh_min) & (sat < cfg_line['dark_max_sat']) & (gray < cfg_line.get('blackhat_max_gray', 140))
    # 흐리게 찍힌 색 구역을 선으로 보지 않게: 정해 둔 색조에 가깝고 채도가 조금이라도 있으면 뺀다
    # (11-27 초록이 채도 34~61로 찍혀 dark_max_sat 55 밑으로 들어와 선으로 잡혔다. 검은 선의 색조는 110 근처)
    for h0, tol, s_min in cfg_line.get('reject_hues', []):
        dh = np.abs(hsv[..., 0].astype(np.int16) - int(h0))
        dh = np.minimum(dh, 180 - dh)
        dark &= ~((dh <= tol) & (sat >= s_min))
    dark = dark.astype(np.uint8)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # 흰 매트 또는 색 구역 (회색 카펫 바닥은 110~140이라 빠진다)
    mat = (gray > 150) | (sat > cfg_line.get('mat_min_sat', 40))
    # 화면 아래 귀퉁이는 어둡게 찍혀 흰 매트가 136~149로 나온다(11-27). 선의 한쪽 옆이 확실한 매트·검정이면 반대쪽은 이 밝기도 매트로 본다
    fg = cfg_line.get('dim_flank_gray', 0)
    dim = (gray > fg) if fg else None
    return dark.astype(bool), (mat, dim)


def runs_full(row_mask, row_mat, min_w, max_w, gap=5, row_dim=None):
    """한 줄의 검은 구간 (시작, 끝, 가운데, 폭). 양옆(gap px 바깥)은 '매트 또는 검정'이어야 한다
    (검정도 허용: 갈림·합류 지점에서 두 선 사이 틈이 좁을 때 둘 다 버리던 문제 — 비전 담당).
    화면 끝에 닿은 구간은 뺀다(광각 렌즈 귀퉁이)."""
    out = []
    xs = np.flatnonzero(row_mask)
    if xs.size == 0:
        return out
    n = row_mask.shape[0]
    breaks = np.flatnonzero(np.diff(xs) > 1)
    starts = np.concatenate(([xs[0]], xs[breaks + 1]))
    ends = np.concatenate((xs[breaks], [xs[-1]]))
    for s, e in zip(starts, ends):
        w = e - s + 1
        if not (min_w <= w <= max_w):
            continue
        lx, rx = s - gap, e + gap
        if lx < 0 or rx >= n:
            continue
        if not ((row_mat[lx] or row_mask[lx]) and (row_mat[rx] or row_mask[rx])):
            # 한쪽 옆이 확실한 매트·검정이고 반대쪽이 조금 어두운 매트면 받는다 (양쪽 다 어두우면 그림자 -> 버림)
            l_ok, r_ok = row_mat[lx] or row_mask[lx], row_mat[rx] or row_mask[rx]
            if row_dim is None or not ((l_ok and row_dim[rx]) or (r_ok and row_dim[lx])):
                continue
        out.append((int(s), int(e), (s + e) / 2.0, int(w)))
    return out


class FloorLineTracker(object):
    """차체 중심 기준 추적 (2026-11-25 에이전트 회의 결론).

    1. 아래 줄부터 위로 올라가며 다음 줄의 선 위치를 예측(x + 기울기·Δ줄)하고, 문(gate) 안의 후보만 본다
    2. 후보가 둘 이상이면 구간 규칙으로 고른다: left(가장 왼쪽) / right(가장 오른쪽) / straight·none(예측에 가장 가까운 것)
       폭이 기대 폭의 1.6배를 넘는 붙은 구간은 규칙 쪽 끝을 기준으로 가운데를 잡는다(갈림·합류에서 갈래 쪽으로 끌려가지 않게)
    3. 고른 점을 바닥 좌표(cm)로 옮겨 가까운 절반으로 직선 y = a + b·x 를 맞춘다
       -> e_cm = a (차체 중심의 옆 오차), psi = atan(b) (방향 차이)
    4. 먼 절반의 방향 psi_far가 가까운 쪽보다 corner_trigger_deg 넘게 왼쪽이면 모서리(반시계라 모두 좌회전).
       두 직선이 만나는 곳 = bend_x (차체 중심에서 꺾이기 시작하는 곳까지 거리)
    """

    name = 'floor'

    def __init__(self, cfg):
        from floor import FloorMap, fit_line
        self.c = cfg['line']
        self.t = cfg['track']
        self.fm = FloorMap(cfg)
        self.fit = fit_line
        self.rule = 'none'
        self._wexp = {}
        self.rows = self._make_rows(cfg['camera']['height'], cfg['camera']['width'])
        self.last_mask = None                # 마지막 사진의 선 마스크 (선을 못 찾아도 남긴다 — 아랫줄 전용 검출기용)
        self.reset()

    def _make_rows(self, h, w):
        """바닥 거리 max_floor_x_cm 안쪽 줄만 쓴다 (먼 줄은 어안 휨·호모그래피 오차가 커서 꺾임으로 오인 — 11-25 회의: 영상 처리)"""
        t = self.t
        bottom = int(h * t['band_bottom']) - 1
        top = int(h * t['band_top'])
        max_x = t.get('max_floor_x_cm', 0)
        if max_x:
            for y in range(bottom, top, -1):
                if self.fm.to_floor([[w / 2.0, y]])[0][0] > max_x:
                    top = y + 1
                    break
        return np.linspace(bottom, top, t['rows']).astype(int)

    def reset(self):
        self.prev_x0 = None
        self.prev_slope = 0.0

    def _width(self, cx, r):
        k = (int(cx) // 16, r)
        if k not in self._wexp:
            try:
                self._wexp[k] = self.fm.expected_width_px(cx, r)
            except Exception:
                self._wexp[k] = 0.0
        return self._wexp[k]

    def _pick(self, cands, x_pred, rule, r, slope=0.0):
        xs = []
        for s, e, cx, w in cands:
            # 비스듬한 선은 가로로 더 넓게 잘린다(45°면 1.4배). 그만큼은 '두 선이 붙은 것'으로 보지 않는다
            # (11-26 1차: 모서리의 비스듬한 선을 두 선으로 보고 'right'로 오른쪽 끝을 잡았다)
            wexp = self._width(cx, r) * math.sqrt(1.0 + slope * slope)
            if wexp > 2 and w > 1.6 * wexp:   # 두 선이 붙은 구간
                lo, hi = s + wexp / 2.0, e - wexp / 2.0
                cx = hi if rule == 'right' else lo if rule == 'left' else min(max(x_pred, lo), hi)
            xs.append(cx)
        if len(xs) == 1:
            return xs[0]
        if rule == 'left':
            return min(xs)
        if rule == 'right':
            return max(xs)
        return min(xs, key=lambda v: abs(v - x_pred))

    def _merge(self, cands, r):
        """한 선이 반사광 띠로 조각난 것(11-25 실측: 22px 선이 85~94 / 102~109로 갈림)을 다시 하나로 붙인다.
        이웃 조각을 합친 폭이 그 줄의 선 폭 1.3배 이하면 한 선으로 본다."""
        if len(cands) < 2:
            return cands
        out = [list(cands[0])]
        for s, e, cx, w in cands[1:]:
            ps = out[-1][0]
            wexp = self._width((ps + e) / 2.0, r)
            if wexp > 2 and e - ps <= 1.3 * wexp:
                out[-1] = [ps, e, (ps + e) / 2.0, e - ps + 1]
            else:
                out.append([s, e, cx, w])
        return [tuple(o) for o in out]

    def _scan_best(self, rows, cand_rows, w, rule):
        """처음부터 다시 찾을 때: 아래 몇 줄의 후보마다 따라가 보고 가장 길게 이어지는 것을 고른다.
        (11-25: 화면 가운데에서 가장 가까운 후보를 고르니 AGV 자기 그림자(폭 50px)를 선으로 잡았다)"""
        starts = []
        for r, cands in list(zip(rows, cand_rows))[:8]:
            for cd in self._merge(cands, r):
                wexp = self._width(cd[2], r)
                if wexp <= 2 or cd[3] <= 1.6 * wexp:      # 선 폭의 1.6배 넘는 덩어리(그림자)는 출발점으로 안 씀
                    starts.append(cd[2])
        results = []
        for x0 in starts or [w / 2.0]:
            picks, slope = self._scan(rows, cand_rows, x0, 0.0, 6, rule)
            if picks:
                results.append((picks, slope))
        if not results:
            return [], 0.0
        # 길이가 비슷한(2점 차 안) 선이 여럿이면 구간 규칙으로 고른다. 규칙이 없으면 화면 가운데에 가까운 것
        # (11-26 회의: 가장 긴 것만 고르면 노랑 뒤 갈림길에서 왼쪽 곁길이 이겼다)
        n = max(len(p) for p, _ in results)
        good = [ps for ps in results if len(ps[0]) >= n - 2]
        if rule == 'right':
            return max(good, key=lambda ps: ps[0][0][0])
        if rule == 'left':
            return min(good, key=lambda ps: ps[0][0][0])
        return min(good, key=lambda ps: abs(ps[0][0][0] - w / 2.0))

    def _scan(self, rows, cand_rows, x0, slope0, first_gate, rule):
        """아래 줄부터 위로: 예측(x + 기울기·Δ줄) 근처 후보만 보고, 둘 이상이면 규칙으로 고른다"""
        t = self.t
        x, slope = x0, slope0
        picks, miss, r_prev = [], 0, None
        for r, cands in zip(rows, cand_rows):
            cands = self._merge(cands, r)
            if r_prev is None:
                x_pred, gate = x, first_gate
            else:
                x_pred = x + slope * (r - r_prev)
                gate = t['gate_px'] + t['gate_per_row'] * abs(r - r_prev)
            near = [cd for cd in cands if abs(cd[2] - x_pred) <= gate + cd[3] / 2.0]
            if not near:
                miss += 1
                if picks and miss > self.t.get('max_miss_rows', 4):   # 반사광 띠로 3~4줄 연속 안 보일 때가 있다(11-25)
                    break
                continue
            xc = self._pick(near, x_pred, rule, r, slope)
            if r_prev is not None:
                # 노랑 X자(straight): 후보가 둘이거나 두 선이 붙은 줄에서는 기울기를 안 바꾼다
                # (11-26 회의: 교차점에서 기울기가 대각선 쪽으로 끌려가 곁길로 갈아탔다)
                crossing = rule == 'straight' and (len(near) > 1 or any(
                    self._width(cd[2], r) > 2 and cd[3] > 1.6 * self._width(cd[2], r) for cd in near))
                if not crossing:
                    slope = 0.5 * slope + 0.5 * (xc - x) / float(r - r_prev)
            x, r_prev, miss = xc, r, 0
            picks.append((xc, r))
        return picks, slope

    def _find_bend(self, pts):
        """왼쪽 꺾임(반시계 모서리)이 시작되는 점의 번호, 없으면 None. (11-25 회의: 영상 처리 담당 기준)
        - 기준선: 가까운 쪽 점 base_points개 이상 + 깊이 base_depth_cm 이상으로 맞춘다 (점 3개·2cm로는 1px 잡음이 꺾임이 됐다)
        - 그 뒤 연속 bend_run개 이상이 왼쪽으로 bend_dev_cm 넘게 벗어나고, 갈수록 더 벗어나며, 마지막이 bend_last_cm 이상
        - 벗어난 구간의 방향이 기준선보다 corner_trigger_deg 이상 왼쪽"""
        t = self.t
        n = len(pts)
        k = t.get('base_points', 5)
        while k < n and pts[k - 1][0] - pts[0][0] < t.get('base_depth_cm', 4.0):
            k += 1
        if n - k < t.get('bend_run', 3):
            return None
        base = self.fit(pts[:k])
        if base is None:
            return None
        a0, b0 = base
        dev = pts[:, 1] - (a0 + b0 * pts[:, 0])
        thr = t.get('bend_dev_cm', 1.0)
        run = t.get('bend_run', 3)
        for i in range(k, n - run + 1):
            seg = dev[i:]
            if all(seg[j] > thr for j in range(run)) and all(seg[j + 1] >= seg[j] - 0.3 for j in range(len(seg) - 1)) \
                    and dev[-1] >= t.get('bend_last_cm', 2.5):
                far = self.fit(pts[i - 1:])
                if far is not None and math.degrees(math.atan(far[1]) - math.atan(b0)) >= t['corner_trigger_deg']:
                    return i
                return None
        return None

    def track(self, img, rule=None):
        rule = rule or self.rule
        t, c = self.t, self.c
        h, w = img.shape[:2]
        mask, (mat, dim) = dark_line_mask(img, c)
        self.last_mask = mask                # 아랫줄 전용 검출기(situ.NearRow)가 다시 쓴다
        rows = self.rows
        # 어두운 귀퉁이 후보 받기는 정해 둔 구간 규칙에서만 켠다 (기본 'left' = 빨강->노랑. 다른 구간은 예전과 똑같이)
        use_dim = dim is not None and rule in c.get('dim_flank_rules', ['left'])
        cand_rows = [runs_full(mask[r], mat[r], c['min_width_px'], c['max_width_px'],
                               row_dim=dim[r] if use_dim else None) for r in rows]
        if self.prev_x0 is not None:
            # prev_slope는 지난 사진의 '맨 위 줄' 기울기다. X자에서는 그걸 맨 아래 줄에 쓰면 대각선으로 끌려간다 -> 0에서 시작
            slope0 = 0.0 if rule == 'straight' else self.prev_slope
            picks, slope = self._scan(rows, cand_rows, self.prev_x0, slope0, t['gate_first_px'], rule)
            if len(picks) < t.get('min_points', 5):   # 직전 위치 근처에 없으면 화면 전체에서 다시 찾는다
                picks, slope = self._scan_best(rows, cand_rows, w, rule)
        else:
            picks, slope = self._scan_best(rows, cand_rows, w, rule)
        if len(picks) < t.get('min_points', 5):
            self.reset()
            return LineResult(False, info={'rows': len(picks), 'rule': rule})
        self.prev_x0 = picks[0][0]
        self.prev_slope = slope

        pts = self.fm.to_floor(picks)
        res = LineResult(True, info={'rows': len(picks), 'rule': rule})
        res.pts = picks
        res.floor_pts = pts                  # 차체 좌표 cm (선 기억용)
        res.cx, res.cy = int(picks[-1][0]), int(picks[-1][1])
        bend_i = self._find_bend(pts)
        straight = pts[:bend_i] if bend_i is not None else pts
        # 11-25 모서리 시험: 먼 쪽 커브까지 넣어 맞추면 ψ가 15~50°로 커져 미리 꺾었다 -> 가까운 점(near_fit_x_cm 안)만으로 맞춘다
        nx = t.get('near_fit_x_cm', 0)
        if nx > 0:
            near = straight[straight[:, 0] <= nx]
            if len(near) >= t.get('near_min_points', 4) and near[-1][0] - near[0][0] >= 2.0:
                straight = near
        a, b = self.fit(straight) or self.fit(pts)     # 꺾이기 전 구간만으로 옆 오차·방향 (커브를 미리 따라 파고들지 않게)
        res.e_cm, res.psi = a, math.atan(b)
        if bend_i is not None:
            res.bend = True
            res.bend_x = float(pts[bend_i - 1][0] + pts[bend_i][0]) / 2.0
            far_fit = self.fit(pts[bend_i - 1:])
            res.psi_far = math.atan(far_fit[1]) if far_fit else res.psi
        res.offset = max(-1.0, min(1.0, -a / 10.0))          # 화면 표시용 (옛 방식과 같은 부호: +면 오른쪽으로)
        res.direction = res.offset
        res.info.update(e_cm=round(a, 2), psi_deg=round(math.degrees(res.psi), 1),
                        psi_far_deg=round(math.degrees(res.psi_far), 1), bend_x=res.bend_x)
        return res
