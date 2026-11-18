"""관제 HMI 화면 시안 (7인치 800x480).

PySide6가 있으면 PySide6로, 없으면 PySide2로 실행한다(라즈베리파이 OS Bookworm은 apt로 PySide2만 설치된다).
데이터는 예시 값이다. 실제로는 현장 기록장(SQLite)과 MQTT 알림을 읽어 채운다.

실행
  python3 hmi_demo.py               창으로 띄우기 (800x480)
  python3 hmi_demo.py --fullscreen  라즈베리파이 7인치 화면에 전체 화면으로
  python3 hmi_demo.py --shot out    두 화면을 PNG로 저장 (화면 없이 하려면 QT_QPA_PLATFORM=offscreen)
조작: 스페이스 = 평소 / 이상 화면 바꾸기, Esc = 종료
"""
import argparse
import os
import sys
import time

ap = argparse.ArgumentParser()
ap.add_argument("--fullscreen", action="store_true")
ap.add_argument("--shot", metavar="DIR")
ap.add_argument("--state", choices=["normal", "alarm"], default="normal")
ARGS = ap.parse_args()
if ARGS.shot:
    os.environ.setdefault("QT_SCALE_FACTOR", "2")  # 캡처만 2배로 선명하게. 배치는 800x480 그대로

try:
    from PySide6 import QtCore, QtGui, QtWidgets
    QT_API = "PySide6"
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets
    QT_API = "PySide2"

Qt, QRectF, QPointF, QTimer = QtCore.Qt, QtCore.QRectF, QtCore.QPointF, QtCore.QTimer
QPainter, QColor, QPen, QFont, QBrush = QtGui.QPainter, QtGui.QColor, QtGui.QPen, QtGui.QFont, QtGui.QBrush
QPolygonF, QFontDatabase = QtGui.QPolygonF, QtGui.QFontDatabase
W = QtWidgets

HERE = os.path.dirname(os.path.abspath(__file__))
C = {"bg": "#1b2026", "top": "#14181d", "panel": "#242a32", "line": "#363e48", "text": "#e6e9ed",
     "sub": "#8d98a5", "dim": "#5c6670", "idle": "#77828e", "work": "#f2c94c", "done": "#3fb950",
     "fault": "#f85149", "faultbg": "#3b2023", "warn": "#f0a03c", "agv": "#58a6ff"}
STATE_TXT = {"idle": "대기", "work": "작업 중", "done": "완료", "fault": "이상", "offline": "끊김"}
STATE_COL = {"idle": C["idle"], "work": C["work"], "done": C["done"], "fault": C["fault"], "offline": C["dim"]}

QSS = """
QWidget#root { background: @bg; }
QLabel { color: @text; background: transparent; }
QFrame#topbar { background: @top; border-bottom: 1px solid @line; }
QLabel#title { font-size: 13px; font-weight: 700; }
QLabel#run { color: @sub; font-size: 11px; }
QLabel#clock { font-size: 15px; font-weight: 700; }
QLabel[role="chip"] { background: @panel; border: 1px solid @line; border-radius: 10px; padding: 2px 9px; font-size: 11px; }
QLabel[role="chip"][tone="bad"] { background: @faultbg; border: 1px solid @fault; color: #ffc2bd; font-weight: 700; }
QLabel[role="mode"] { background: #1f3550; border: 1px solid #2f5a8a; color: #bcd8ff; border-radius: 10px; padding: 2px 9px; font-size: 11px; font-weight: 700; }
QFrame[role="card"] { background: @panel; border: 1px solid @line; border-radius: 7px; }
QFrame[role="card"][state="fault"] { background: @faultbg; border: 1px solid @fault; }
QFrame[role="card"][state="offline"] { background: #1e2329; border: 1px dashed @dim; }
QFrame[role="bar"] { border: none; border-radius: 2px; background: @line; }
QFrame[role="bar"][state="work"] { background: @work; }
QFrame[role="bar"][state="done"] { background: @done; }
QFrame[role="bar"][state="fault"] { background: @fault; }
QFrame[role="bar"][state="offline"] { background: transparent; }
QLabel[role="code"] { color: @sub; font-size: 11px; font-weight: 700; }
QLabel[role="name"] { font-size: 12px; font-weight: 700; }
QLabel[role="state"] { font-size: 19px; font-weight: 700; color: @idle; }
QLabel[role="state"][state="work"] { color: @work; }
QLabel[role="state"][state="done"] { color: @done; }
QLabel[role="state"][state="fault"] { color: #ff7b72; }
QLabel[role="state"][state="offline"] { color: @dim; }
QLabel[role="body"] { font-size: 11px; }
QLabel[role="foot"] { color: @sub; font-size: 10px; }
QLabel[role="foot"][tone="bad"] { color: #ff9b94; font-weight: 700; }
QProgressBar { background: @bg; border: none; border-radius: 2px; }
QProgressBar::chunk { background: @work; border-radius: 2px; }
QFrame[role="panel"] { background: @panel; border: 1px solid @line; border-radius: 7px; }
QFrame[role="panel"][tone="bad"] { background: @faultbg; border: 1px solid @fault; }
QLabel[role="ph"] { color: @sub; font-size: 11px; font-weight: 700; }
QLabel[role="ph"][tone="bad"] { color: #ff9b94; font-size: 12px; }
QLabel[role="cap"] { color: @dim; font-size: 10px; }
QLabel[role="big"] { font-size: 24px; font-weight: 700; }
QLabel[role="big"][tone="warn"] { color: @warn; }
QLabel[role="bigcap"] { color: @sub; font-size: 10px; }
QLabel[role="note"] { color: @sub; font-size: 10px; }
QLabel[role="alarm"] { font-size: 11px; background: #2a1719; border: 1px solid #5a2a2d; border-radius: 5px; padding: 4px 6px; }
QTableWidget { background-color: @panel; border: none; color: @text; font-size: 11px; gridline-color: @panel; }
QTableWidget::item { padding: 0 4px; border-bottom: 1px solid #2c333c; }
QHeaderView::section { background-color: @panel; color: @sub; border: none; border-bottom: 1px solid @line; padding: 2px 4px; font-size: 10px; font-weight: 700; }
QPushButton { background: #2e3540; color: @text; border: 1px solid @line; border-radius: 6px; padding: 7px 4px; font-size: 11px; font-weight: 700; }
QPushButton:pressed { background: #3a4350; }
QPushButton[kind="danger"] { background: #5a2226; border: 1px solid @fault; color: #ffd6d2; }
"""
for _k in sorted(C, key=len, reverse=True):
    QSS = QSS.replace("@" + _k, C[_k])

FONT_FAMILY = "sans-serif"


def pick_font():
    """한글 글꼴을 고른다. 같은 폴더의 fonts/ 안 글꼴 파일을 먼저 불러온다."""
    global FONT_FAMILY
    files = [os.path.join(HERE, "fonts", f) for f in sorted(os.listdir(os.path.join(HERE, "fonts")))] \
        if os.path.isdir(os.path.join(HERE, "fonts")) else []
    files += ["C:/Windows/Fonts/malgun.ttf", "C:/Windows/Fonts/malgunbd.ttf"]
    for fp in files:
        if os.path.isfile(fp) and fp.lower().endswith((".ttf", ".otf", ".ttc")):
            QFontDatabase.addApplicationFont(fp)
    try:
        families = QFontDatabase.families()        # PySide6
    except TypeError:
        families = QFontDatabase().families()      # PySide2
    for name in ("Malgun Gothic", "NanumGothic", "NanumBarunGothic", "Noto Sans CJK KR", "Noto Sans KR",
                 "UnDotum", "Baekmuk Gulim"):
        if name in families:
            FONT_FAMILY = name
            return name
    print("경고: 한글 글꼴을 찾지 못했습니다. 'sudo apt install fonts-nanum' 후 다시 실행하세요.")
    return None


def lab(text, **props):
    w = W.QLabel(text)
    for k, v in props.items():
        if v is not None:
            w.setProperty(k, v)
    return w


def font(px, bold=False):
    f = QFont(FONT_FAMILY)
    f.setPixelSize(px)
    f.setBold(bold)
    return f


class Track(W.QWidget):
    """AGV가 지금 어느 공정에 있는지 보여 주는 줄."""

    def __init__(self, stations, pos, lines, tone=None):
        super().__init__()
        self.stations, self.pos, self.lines, self.tone = stations, pos, lines, tone
        self.setFixedHeight(58)

    def paintEvent(self, e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor(C["line"]), 1))
        p.setBrush(QColor(C["panel"]))
        p.drawRoundedRect(QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5), 7, 7)
        left = Qt.AlignLeft | Qt.AlignVCenter
        p.setFont(font(12, True)); p.setPen(QColor(C["text"]))
        p.drawText(QRectF(12, 6, 175, 17), left, self.lines[0])
        p.setFont(font(10, True)); p.setPen(QColor("#ff9b94" if self.tone == "bad" else C["work"]))
        p.drawText(QRectF(12, 23, 175, 15), left, self.lines[1])
        p.setFont(font(10)); p.setPen(QColor(C["sub"]))
        p.drawText(QRectF(12, 38, 175, 15), left, self.lines[2])
        x0, x1, y = 222, self.width() - 34, 27
        p.setPen(QPen(QColor("#46505b"), 3, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(x0 - 16, y), QPointF(x1 + 16, y))
        n = len(self.stations)
        xs = [x0 + (x1 - x0) * i / (n - 1) for i in range(n)]
        p.setFont(font(10, True))
        for (name, st), x in zip(self.stations, xs):
            col = QColor(STATE_COL[st])
            if st == "offline":
                p.setPen(QPen(col, 1.5, Qt.DashLine)); p.setBrush(QColor(C["panel"]))
            else:
                p.setPen(Qt.NoPen); p.setBrush(col)
            p.drawEllipse(QPointF(x, y), 5.5, 5.5)
            p.setPen(col if st in ("work", "fault") else QColor(C["sub"]))
            p.drawText(QRectF(x - 40, 3, 80, 14), Qt.AlignCenter, name)
        ax = x0 + (x1 - x0) * self.pos / (n - 1)
        p.setPen(Qt.NoPen); p.setBrush(QColor(C["agv"]))
        p.drawPolygon(QPolygonF([QPointF(ax, y + 7), QPointF(ax - 4, y + 12), QPointF(ax + 4, y + 12)]))
        p.drawRoundedRect(QRectF(ax - 19, y + 12, 38, 15), 4, 4)
        p.setPen(QColor("#0b1a2e"))
        p.drawText(QRectF(ax - 19, y + 12, 38, 15), Qt.AlignCenter, "AGV")
        p.end()


def topbar(d, clocks):
    f = W.QFrame(); f.setObjectName("topbar"); f.setFixedHeight(38)
    h = W.QHBoxLayout(f); h.setContentsMargins(12, 0, 12, 0); h.setSpacing(7)
    t = W.QLabel("차체 라인 관제"); t.setObjectName("title"); h.addWidget(t)
    r = W.QLabel(d["run"]); r.setObjectName("run"); h.addWidget(r)
    h.addStretch(1)
    for text, tone in d["chips"]:
        c = lab(text, role="chip", tone=tone); c.setTextFormat(Qt.RichText); h.addWidget(c, 0, Qt.AlignVCenter)
    h.addWidget(lab("AGV 모드", role="mode"), 0, Qt.AlignVCenter)
    ck = W.QLabel(d["clock"]); ck.setObjectName("clock"); h.addWidget(ck)
    clocks.append(ck)
    return f


def card(s):
    f = W.QFrame(); f.setProperty("role", "card"); f.setProperty("state", s["state"]); f.setFixedHeight(122)
    f.setSizePolicy(W.QSizePolicy.Ignored, W.QSizePolicy.Fixed)
    v = W.QVBoxLayout(f); v.setContentsMargins(10, 8, 10, 8); v.setSpacing(3)
    bar = W.QFrame(); bar.setProperty("role", "bar"); bar.setProperty("state", s["state"]); bar.setFixedHeight(4)
    v.addWidget(bar)
    hd = W.QHBoxLayout(); hd.setSpacing(5)
    if s.get("code"):
        hd.addWidget(lab(s["code"], role="code"))
    hd.addWidget(lab(s["name"], role="name")); hd.addStretch(1)
    v.addLayout(hd)
    v.addWidget(lab(s.get("label", STATE_TXT[s["state"]]), role="state", state=s["state"]))
    v.addWidget(lab(s["body"], role="body"))
    if "prog" in s:
        pb = W.QProgressBar(); pb.setRange(0, 100); pb.setValue(s["prog"]); pb.setTextVisible(False); pb.setFixedHeight(4)
        v.addWidget(pb)
    v.addStretch(1)
    v.addWidget(lab(s["foot"], role="foot", tone=s.get("tone")))
    return f


def events_panel(rows):
    f = W.QFrame(); f.setProperty("role", "panel")
    v = W.QVBoxLayout(f); v.setContentsMargins(10, 6, 8, 4); v.setSpacing(2)
    hd = W.QHBoxLayout(); hd.addWidget(lab("최근 기록", role="ph")); hd.addStretch(1)
    hd.addWidget(lab("현장 기록장(SQLite)을 읽어 표시", role="cap")); v.addLayout(hd)
    t = W.QTableWidget(len(rows), 4)
    t.setHorizontalHeaderLabels(["순번", "시각", "내용", "서버"])
    t.verticalHeader().setVisible(False); t.verticalHeader().setDefaultSectionSize(21)
    t.setShowGrid(False); t.setFocusPolicy(Qt.NoFocus); t.setSelectionMode(W.QAbstractItemView.NoSelection)
    t.setEditTriggers(W.QAbstractItemView.NoEditTriggers); t.setFrameShape(W.QFrame.NoFrame)
    t.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff); t.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    hh = t.horizontalHeader(); hh.setHighlightSections(False); hh.setDefaultAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    hh.setFixedHeight(20)
    for i, wd in enumerate([46, 62, 0, 58]):
        if wd:
            hh.setSectionResizeMode(i, W.QHeaderView.Fixed); t.setColumnWidth(i, wd)
        else:
            hh.setSectionResizeMode(i, W.QHeaderView.Stretch)
    for r, (seq, tm, txt, sent, tone) in enumerate(rows):
        for c, val in enumerate([str(seq), tm, txt, "넘김" if sent else "안 넘김"]):
            it = W.QTableWidgetItem(val)
            col = C["sub"] if c in (0, 1) else C["text"]
            if c == 2 and tone == "bad":
                col = "#ff9b94"
            if c == 3:
                col = C["dim"] if sent else C["warn"]
                if not sent:
                    it.setFont(font(11, True))
            it.setForeground(QBrush(QColor(col)))
            t.setItem(r, c, it)
    v.addWidget(t)
    return f


def side_panel(s):
    f = W.QFrame(); f.setProperty("role", "panel"); f.setProperty("tone", s.get("tone")); f.setFixedWidth(238)
    v = W.QVBoxLayout(f); v.setContentsMargins(10, 7, 10, 8); v.setSpacing(5)
    v.addWidget(lab(s["title"], role="ph", tone=s.get("tone")))
    if "nums" in s:
        row = W.QHBoxLayout(); row.setSpacing(4)
        for num, cap, tone in s["nums"]:
            col = W.QVBoxLayout(); col.setSpacing(0)
            col.addWidget(lab(num, role="big", tone=tone), 0, Qt.AlignHCenter)
            col.addWidget(lab(cap, role="bigcap"), 0, Qt.AlignHCenter)
            row.addLayout(col)
        v.addLayout(row)
        v.addWidget(lab(s["note"], role="note"), 0, Qt.AlignHCenter)
    if "devices" in s:
        v.addSpacing(4)
        v.addWidget(lab("장치 연결", role="ph"))
        g2 = W.QGridLayout(); g2.setHorizontalSpacing(8); g2.setVerticalSpacing(2)
        for i, (dn, ok) in enumerate(s["devices"]):
            dot = C["done"] if ok else C["dim"]
            w = lab("<span style='color:%s'>●</span> %s" % (dot, dn), role="note"); w.setTextFormat(Qt.RichText)
            g2.addWidget(w, i // 4, i % 4)
        v.addLayout(g2)
    for a in s.get("alarms", []):
        w = lab(a, role="alarm"); w.setTextFormat(Qt.RichText); v.addWidget(w)
    v.addStretch(1)
    g = W.QGridLayout(); g.setSpacing(5)
    for i, (text, kind) in enumerate(s["buttons"]):
        b = W.QPushButton(text); b.setFocusPolicy(Qt.NoFocus)
        if kind:
            b.setProperty("kind", kind)
        g.addWidget(b, i // 2, i % 2)
    v.addLayout(g)
    return f


class Screen(W.QWidget):
    """화면 한 장(평소 또는 이상)."""

    def __init__(self, d, clocks):
        super().__init__()
        self.setObjectName("root"); self.setAttribute(Qt.WA_StyledBackground, True)
        root = W.QVBoxLayout(self); root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)
        root.addWidget(topbar(d, clocks))
        body = W.QVBoxLayout(); body.setContentsMargins(8, 8, 8, 8); body.setSpacing(6)
        root.addLayout(body)
        row = W.QHBoxLayout(); row.setSpacing(6)
        for s in d["stations"]:
            row.addWidget(card(s), 1)
        body.addLayout(row)
        track_st = [((s.get("code", "") + " " + s["name"]).strip(), s["state"]) for s in d["stations"]]
        body.addWidget(Track(track_st, d["agv_pos"], d["agv"], d.get("agv_tone")))
        bottom = W.QHBoxLayout(); bottom.setSpacing(6)
        bottom.addWidget(events_panel(d["events"]), 1)
        bottom.addWidget(side_panel(d["side"]), 0)
        body.addLayout(bottom, 1)


class HMI(W.QStackedWidget):
    def __init__(self, live_clock=True):
        super().__init__()
        self.setWindowTitle("관제 HMI (%s)" % QT_API)
        self.clocks = []
        self.addWidget(Screen(NORMAL, self.clocks))
        self.addWidget(Screen(ALARM, self.clocks))
        if live_clock:
            self.timer = QTimer(self); self.timer.timeout.connect(self.tick); self.timer.start(1000); self.tick()

    def tick(self):
        now = time.strftime("%H:%M:%S")
        for c in self.clocks:
            c.setText(now)

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Space:
            self.setCurrentIndex(1 - self.currentIndex())
        elif e.key() == Qt.Key_Escape:
            self.close()


DOT_OK = "<span style='color:#3fb950'>●</span> "
DOT_BAD = "<span style='color:#ff6b61'>●</span> "
NORMAL = dict(
    run="회차 r-20261201-1405", clock="14:23:05",
    chips=[(DOT_OK + "브로커", None), (DOT_OK + "서버 · 미전달 0", None)],
    stations=[
        dict(code="P1", name="프레스", state="idle", body="–", foot="최근 BODY-002 · 14:22"),
        dict(code="P2", name="차체", state="work", body="BODY-002", prog=60, foot="재고 7 · 6/10초"),
        dict(code="P3", name="도장", state="idle", body="–", foot="습도 48%"),
        dict(code="P4", name="의장", state="idle", body="–", foot="부품 재고 3"),
        dict(name="검사", state="idle", body="–", foot="오늘 합격 11 · 보류 1"),
    ],
    agv_pos=1, agv=("AGV-1 · 자동", "P2 정차 · 작업 중", "BODY-002 · 배터리 78%"),
    events=[
        (1047, "14:23:01", "P2 작업 허가 · BODY-002", True, None),
        (1046, "14:23:00", "AGV 멈춤 완료 · P2", True, None),
        (1045, "14:22:59", "P2 태그 읽음 · BODY-002", True, None),
        (1044, "14:22:41", "P1 작업 끝 · BODY-002", True, None),
        (1043, "14:22:31", "P1 작업 허가 · BODY-002", True, None),
        (1042, "14:22:30", "AGV 멈춤 완료 · P1", True, None),
        (1041, "14:22:29", "P1 태그 읽음 · BODY-002", True, None),
        (1040, "14:22:05", "검사 합격 · BODY-001", True, None),
    ],
    side=dict(title="오늘 생산", nums=[("11", "완료", None), ("1", "보류", "warn"), ("1", "진행 중", None)],
              note="다음 지시  BODY-003",
              devices=[("P1", 1), ("P2", 1), ("P3", 1), ("P4", 1), ("검사", 1), ("AGV", 1), ("태그", 1)],
              buttons=[("AGV 수동 조작", None), ("검사 정정", None), ("부품 부족 띄우기", None), ("새 회차", None)]),
)
ALARM = dict(
    run="회차 r-20261201-1405", clock="14:23:40",
    chips=[(DOT_OK + "브로커", None), (DOT_BAD + "서버 끊김 · 미전달 3", "bad")],
    stations=[
        dict(code="P1", name="프레스", state="idle", body="–", foot="최근 BODY-002 · 14:22"),
        dict(code="P2", name="차체", state="idle", body="–", foot="재고 6 · 최근 BODY-002"),
        dict(code="P3", name="도장", state="fault", label="이상 · 감지", body="BODY-002", foot="습도 82% (기준 70%)", tone="bad"),
        dict(code="P4", name="의장", state="offline", body="–", foot="마지막 수신 14:23:21"),
        dict(name="검사", state="idle", body="–", foot="오늘 합격 11 · 보류 1"),
    ],
    agv_pos=2, agv=("AGV-1 · 정지", "P3 이상 해결 대기", "BODY-002 · 배터리 77%"), agv_tone="bad",
    events=[
        (1054, "14:23:33", "P3 이상 감지 · 습도 82%", False, "bad"),
        (1053, "14:23:27", "P4 연결 끊김", False, "bad"),
        (1052, "14:23:21", "P3 작업 허가 · BODY-002", False, None),
        (1051, "14:23:20", "AGV 멈춤 완료 · P3", True, None),
        (1050, "14:23:19", "P3 태그 읽음 · BODY-002", True, None),
        (1049, "14:23:12", "AGV 출발 · P2 → P3", True, None),
        (1048, "14:23:11", "P2 작업 끝 · BODY-002", True, None),
        (1047, "14:23:01", "P2 작업 허가 · BODY-002", True, None),
    ],
    side=dict(title="경보 2건", tone="bad",
              alarms=["<b>P3 도장</b> &nbsp;습도 초과 · 감지<br><span style='color:#a7b0ba'>14:23:33 · 점검 전</span>",
                      "<b>P4 의장</b> &nbsp;연결 끊김<br><span style='color:#a7b0ba'>14:23:27 · 6초 동안 신호 없음</span>"],
              buttons=[("점검 시작", "danger"), ("AGV 수동 조작", None), ("검사 정정", None), ("새 회차", None)]),
)


def main():
    app = W.QApplication(sys.argv)
    fam = pick_font()
    app.setFont(font(11))
    app.setStyleSheet(QSS)
    print("Qt 바인딩: %s (Qt %s) · 글꼴: %s" % (QT_API, QtCore.qVersion(), fam))
    if ARGS.shot:
        os.makedirs(ARGS.shot, exist_ok=True)
        hmi = HMI(live_clock=False)
        hmi.setFixedSize(800, 480)
        for i, name in enumerate(("hmi_normal", "hmi_alarm")):
            hmi.setCurrentIndex(i)
            hmi.ensurePolished(); hmi.layout().activate(); hmi.currentWidget().layout().activate()
            pix = hmi.grab()
            path = os.path.join(ARGS.shot, name + ".png")
            pix.save(path)
            print("저장:", path, pix.width(), "x", pix.height())
        return 0
    hmi = HMI()
    hmi.setCurrentIndex(1 if ARGS.state == "alarm" else 0)
    if ARGS.fullscreen:            # 화면 크기에 맞춰 늘어난다 (800x480이 아닌 화면도 그대로)
        hmi.setCursor(Qt.BlankCursor)
        hmi.showFullScreen()
    else:                          # PC에서 개발할 때는 7인치와 같은 크기로 고정
        hmi.setFixedSize(800, 480)
        hmi.show()
    run = app.exec if hasattr(app, "exec") else app.exec_
    return run()


if __name__ == "__main__":
    sys.exit(main())
