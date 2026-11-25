"""mqtt_tool.py — Windows PC에서 ESP32 실습을 돕는 MQTT 도구 (엣지 서버 Pi5 흉내 일부)

설치(한 번):   py -m pip install paho-mqtt
사용 (PowerShell, 이 파일이 있는 폴더에서):
  py mqtt_tool.py watch                         지나가는 factory/# 메시지를 모두 시각과 함께 보기
  py mqtt_tool.py send P2 re=42 ok=1           factory/station/P2/cmd 로 {"re":42,"ok":1} 보내기
                                                (PowerShell 5.1은 큰따옴표를 빼먹어 JSON을 직접 넘기기 어렵다 → 이름=값으로 쓴다)
  py mqtt_tool.py state P2 WORK 7               factory/station/P2/state 에 지금 상태를 보관(retained)으로 보내기
  py mqtt_tool.py reply                         공정이 보낸 report·tag에 {"re":n,"ok":1} 답을 자동으로 돌려주기
옵션: --host 브로커 IP (기본 localhost), --port 1883

reply는 '적고 답한다' 규칙의 흉내다. 진짜 기록 프로그램(엣지 서버 Pi5)이 생기기 전 실습용이다. 아무것도 저장하지 않는다.
"""
import argparse
import json
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt가 없습니다. 먼저:  py -m pip install paho-mqtt")


def make_client(name):
    try:   # paho-mqtt 2.x
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=name)
    except AttributeError:   # 1.x
        return mqtt.Client(client_id=name)


def connect(c, host, port):
    try:
        c.connect(host, port, keepalive=30)
    except OSError as e:
        sys.exit(f"브로커({host}:{port})에 붙지 못했습니다: {e}\n"
                 "mosquitto가 켜져 있는지, IP가 맞는지, 방화벽에서 1883이 열려 있는지 보세요.")


def now():
    return time.strftime("%H:%M:%S")


def cmd_watch(a):
    c = make_client("pc-watch")
    c.on_connect = lambda cl, *_: (cl.subscribe("factory/#"), print(f"{now()} 연결됨 — factory/# 보는 중 (끝내려면 Ctrl+C)"))
    c.on_message = lambda cl, u, m: print(f"{now()} {m.topic}{' [보관]' if m.retain else ''}  {m.payload.decode(errors='replace')}")
    connect(c, a.host, a.port)
    c.loop_forever()


def publish_once(a, topic, payload, retain=False):
    c = make_client("pc-send")
    connect(c, a.host, a.port)
    c.loop_start()
    info = c.publish(topic, payload, qos=1, retain=retain)
    info.wait_for_publish(5)
    c.loop_stop()
    c.disconnect()
    print(f"{now()} 보냄 {topic}{' [보관]' if retain else ''}  {payload}")


def to_value(v):
    try:
        return int(v)
    except ValueError:
        return v


def cmd_send(a):
    obj = {}
    for kv in a.pairs:
        if "=" not in kv:
            sys.exit(f"이름=값 모양이 아닙니다: {kv}  (예: re=42 ok=1)")
        k, v = kv.split("=", 1)
        obj[k] = to_value(v)
    publish_once(a, f"factory/station/{a.station}/cmd", json.dumps(obj, separators=(",", ":")))


def cmd_state(a):
    payload = json.dumps({"st": a.st, "job": a.job, "lamp": a.lamp, "fault": 0}, separators=(",", ":"))
    publish_once(a, f"factory/station/{a.station}/state", payload, retain=True)


def cmd_reply(a):
    c = make_client("pc-reply")

    def on_connect(cl, *_):
        cl.subscribe("factory/station/+/report")
        cl.subscribe("factory/station/+/tag")
        print(f"{now()} 연결됨 — 공정의 report·tag에 자동으로 답합니다 (끝내려면 Ctrl+C)")

    def on_message(cl, u, m):
        station = m.topic.split("/")[2]
        text = m.payload.decode(errors="replace")
        try:
            n = json.loads(text).get("n")
        except ValueError:
            print(f"{now()} {m.topic}  JSON 아님, 답 안 함: {text}")
            return
        print(f"{now()} 받음 {m.topic}  {text}")
        if n is None:
            return
        ans = json.dumps({"re": n, "ok": 1}, separators=(",", ":"))
        cl.publish(f"factory/station/{station}/cmd", ans, qos=1)
        print(f"{now()} 답함 factory/station/{station}/cmd  {ans}")

    c.on_connect, c.on_message = on_connect, on_message
    connect(c, a.host, a.port)
    c.loop_forever()


def main():
    p = argparse.ArgumentParser(description="ESP32 실습용 MQTT 도구")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("watch")
    s = sub.add_parser("send"); s.add_argument("station"); s.add_argument("pairs", nargs="+")
    s = sub.add_parser("state"); s.add_argument("station"); s.add_argument("st"); s.add_argument("job", type=int)
    s.add_argument("--lamp", default="Yb")
    sub.add_parser("reply")
    a = p.parse_args()
    try:
        {"watch": cmd_watch, "send": cmd_send, "state": cmd_state, "reply": cmd_reply}[a.cmd](a)
    except KeyboardInterrupt:
        print("\n끝")


if __name__ == "__main__":
    main()
