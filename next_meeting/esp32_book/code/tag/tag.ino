// tag.ino — 스마트태그 뼈대 (Arduino-ESP32 3.x / PubSubClient 2.8 / ArduinoJson 7)
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "secrets.h"

#define TAG_NAME "tag1"
#define BASE     "factory/tag/" TAG_NAME "/"
#define PIN_LED  2               // NodeMCU-32S 내장 LED
#define PIN_BTN  0               // 보드의 BOOT 버튼 (부팅 후에는 일반 버튼으로 쓸 수 있다). 실물에선 교체
#define LOG_N    16              // 최근 N건만 보관 (링 로그)

struct Rec { uint32_t seq; uint16_t id; char body[16]; char p[4]; char r[4]; };

WiFiClient   net;
PubSubClient mqtt(net);
Preferences  nv;
uint32_t lastSeq = 0, msgN = 0;
uint8_t  head = 0, count = 0;    // head = 다음에 쓸 칸
uint16_t lastId = 0;

// ───── 상태 기계 + LED 패턴 (100ms × 16칸 비트 패턴, delay 없음) ─────
enum State { ST_OFFLINE, ST_IDLE, ST_WROTE, ST_ERROR };
const uint16_t PATTERN[] = { 0x5555,   // OFFLINE: 빠른 깜빡임
                             0x0001,   // IDLE: 1.6초마다 짧게 한 번
                             0xFFFF,   // WROTE: 켜짐
                             0x0005 }; // ERROR: 두 번 깜빡
State state = ST_OFFLINE;
uint32_t stateSince = 0;
void setState(State s) { state = s; stateSince = millis(); }
void serviceLed() { digitalWrite(PIN_LED, (PATTERN[state] >> ((millis() / 100) % 16)) & 1); }

// ───── 플래시 링 로그 (NVS) ─────
void logLoad() {
  nv.begin("tag", false);                     // 이름공간 "tag", 읽기/쓰기
  head = nv.getUChar("head", 0); count = nv.getUChar("cnt", 0); lastSeq = nv.getUInt("seq", 0);
}
void logAdd(const Rec &r) {
  char key[8]; snprintf(key, sizeof key, "r%u", (unsigned)head);
  nv.putBytes(key, &r, sizeof r);             // 기록 먼저, 위치/seq는 그 다음 (중간에 꺼져도 재전송으로 복구)
  head = (head + 1) % LOG_N; if (count < LOG_N) count++;
  nv.putUChar("head", head); nv.putUChar("cnt", count); nv.putUInt("seq", r.seq);
  lastSeq = r.seq;
}
void logDump() {                              // 오래된 것부터 USB로 출력
  for (uint8_t i = 0; i < count; i++) {
    char key[8]; snprintf(key, sizeof key, "r%u", (unsigned)((head + LOG_N - count + i) % LOG_N));
    Rec r;
    if (nv.getBytes(key, &r, sizeof r) == sizeof r)
      Serial.printf("# seq=%lu id=%u body=%s p=%s r=%s\n", (unsigned long)r.seq, (unsigned)r.id, r.body, r.p, r.r);
  }
}

void answer(uint16_t id) {                    // 명령에 대한 답: {"re":5,"ok":1} (신고 번호 없음, 다시 보내지 않는다)
  char out[40];
  snprintf(out, sizeof out, "{\"re\":%u,\"ok\":1}", (unsigned)id);
  mqtt.publish(BASE "report", out);
}
void report(const char *kind) {               // 태그가 먼저 알리는 신고: {"n":3,"k":"BTN"}
  char out[48];
  msgN = msgN % 255 + 1;                      // 신고 번호는 1~255를 돌려 쓴다
  snprintf(out, sizeof out, "{\"n\":%lu,\"k\":\"%s\"}", (unsigned long)msgN, kind);
  mqtt.publish(BASE "report", out);
}

// ───── 명령 수신 ─────
void onMqtt(char *topic, byte *payload, unsigned int len) {
  if (strcmp(topic, BASE "cmd") != 0) return;
  JsonDocument doc;
  if (deserializeJson(doc, (const char *)payload, len)) { setState(ST_ERROR); return; }
  const char *what = doc["do"] | "";          // 키가 없으면 "" (기본값)
  uint16_t id = doc["id"] | 0;
  if (strcmp(what, "WRITE") == 0) {
    Rec r = {};
    r.seq = doc["seq"] | 0u; r.id = id;
    strlcpy(r.body, doc["body"] | "", sizeof r.body);
    strlcpy(r.p, doc["p"] | "", sizeof r.p);
    strlcpy(r.r, doc["r"] | "", sizeof r.r);
    if (count == 0 || r.seq != lastSeq) logAdd(r);   // 같은 seq 재전송이면 저장은 건너뛰고 답만 다시
    lastId = id;
    answer(id);                               // payload는 이미 다 복사했으므로 여기서 publish해도 안전
    setState(ST_WROTE);
  }
}

// ───── 버튼: 짧게/길게 (30ms 디바운스, 눌림 = LOW) ─────
void onShort() { if (mqtt.connected()) report("BTN"); }           // 예시 동작 — 팀 규격으로 교체
void onLong()  { logDump(); }                                     // 예시 동작 — 팀 규격으로 교체
void pollButton() {
  static bool stable = HIGH, last = HIGH, longDone = false;
  static uint32_t tEdge = 0, tDown = 0;
  bool now = digitalRead(PIN_BTN);
  if (now != last) { last = now; tEdge = millis(); }
  if (now != stable && millis() - tEdge >= 30) {
    stable = now;
    if (stable == LOW) { tDown = millis(); longDone = false; }
    else if (!longDone) onShort();            // 길게 누르기가 이미 처리됐으면 뗄 때는 무시
  }
  if (stable == LOW && !longDone && millis() - tDown >= 1500) { longDone = true; onLong(); }
}

// ───── 연결 유지 ─────
void serviceNet() {
  static uint32_t lastWifi = 0, lastMqtt = 0;
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifi >= 15000) { lastWifi = millis(); WiFi.disconnect(); WiFi.begin(WIFI_SSID, WIFI_PASS); }
  } else if (!mqtt.connected() && millis() - lastMqtt >= 3000) {
    lastMqtt = millis();
    if (mqtt.connect("tag-" TAG_NAME, BASE "link", 0, true, "0")) {
      mqtt.publish(BASE "link", "1", true);
      mqtt.subscribe(BASE "cmd");
    } else Serial.printf("# MQTT 실패 rc=%d\n", mqtt.state());
  }
  if (mqtt.connected()) mqtt.loop();

  // 상태 전이
  if (!mqtt.connected()) { if (state != ST_OFFLINE) setState(ST_OFFLINE); }
  else if (state == ST_OFFLINE) setState(ST_IDLE);
  else if ((state == ST_WROTE || state == ST_ERROR) && millis() - stateSince >= 2000) setState(ST_IDLE);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  logLoad();
  Serial.printf("\n# tag %s 시작, 저장 %u건, lastSeq=%lu\n", TAG_NAME, (unsigned)count, (unsigned long)lastSeq);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(4);
  mqtt.setSocketTimeout(3);
}

void loop() {
  serviceNet();
  pollButton();
  serviceLed();
}
