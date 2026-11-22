// modem.ino — 공정 통신 모뎀: 공정보드 UART <-> MQTT
// 대상: Arduino-ESP32 3.x / PubSubClient 2.8 / ArduinoJson 7
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"             // WIFI_SSID, WIFI_PASS, MQTT_HOST, MQTT_PORT

// ───── 설정: 보드마다 STATION만 바꾼다 ─────
#define STATION        "P2" 
#define TEST_MODE      0         // 0 실사용 | 1 USB 시리얼 모니터가 공정보드 역할 | 2 TX2→RX2 점퍼 루프백
#define PIN_RX2        16        // 코어 3.x는 Serial2 기본 핀이 4/25로 바뀌었다 → 반드시 직접 지정
#define PIN_TX2        17
#define LINE_CAP       240       // '$'부터 체크섬까지 한 줄 최대 길이
#define MQTT_BUF       512       // PubSubClient 버퍼 (기본 256은 부족)
#define LINK_REPEAT_MS 2000      // $L(연결 상태)을 2초마다 보드에 알린다. 보드는 5초 동안 못 받으면 끊김으로 본다

#define BASE "factory/station/" STATION "/"
#if TEST_MODE == 1
  #define BOARD Serial           // USB로 줄을 직접 타이핑
#else
  #define BOARD Serial2
#endif
#define DBG(...) Serial.printf(__VA_ARGS__)   // 디버그는 항상 USB, '#'으로 시작

WiFiClient   net;
PubSubClient mqtt(net);

// ───── 체크섬 / 보드로 한 줄 보내기 ─────
uint8_t xorOf(const char *p, size_t n) { uint8_t c = 0; while (n--) c ^= (uint8_t)*p++; return c; }

void sendToBoard(const char *body) {          // body = 종류 글자 + 내용, 예: "C,re=42,ok=1"
  BOARD.printf("$%s*%02X\n", body, xorOf(body, strlen(body)));
  DBG("# ->board $%s\n", body);
}

// ───── 보드 → MQTT ─────
void handleLine(char *line, size_t len) {     // line = "$...*HH" (개행 제거됨)
  if (len < 5 || line[0] != '$' || line[len - 3] != '*') { DBG("# 형식 오류: %s\n", line); return; }
  char *end;
  uint8_t want = (uint8_t)strtoul(line + len - 2, &end, 16);
  if (end != line + len || xorOf(line + 1, len - 4) != want) { DBG("# 체크섬 오류: %s\n", line); return; }

  char type = line[1];
  const uint8_t *payload = (const uint8_t *)line + 2;   // 종류 글자 다음부터 '*' 앞까지
  size_t plen = len - 5;
  const char *topic = type == 'T' ? BASE "tag" : type == 'R' ? BASE "report" : type == 'H' ? BASE "alive" : nullptr;
  if (!topic) { DBG(TEST_MODE == 2 ? "# 루프백 OK: %s\n" : "# 모르는 종류: %s\n", line); return; }
  if (!mqtt.connected()) { DBG("# 링크 없음, 버림 (보드가 재전송)\n"); return; }
  bool ok = mqtt.publish(topic, payload, plen, false);  // JSON을 그대로, QoS 0
  DBG("# ->mqtt %s %s\n", topic, ok ? "ok" : "실패(버퍼 초과?)");
}

void pollBoard() {                            // 한 글자씩 모아서 '\n'에서 한 줄 완성. 절대 기다리지 않는다
  static char buf[LINE_CAP + 1];
  static size_t n = 0;
  static bool over = false;
  while (BOARD.available()) {
    char c = (char)BOARD.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (over) DBG("# 줄이 너무 김, 버림\n");
      else if (n) { buf[n] = 0; handleLine(buf, n); }
      n = 0; over = false;
    } else if (n < LINE_CAP) buf[n++] = c;
    else over = true;                         // 길이 초과: 다음 '\n'까지 통째로 버린다
  }
}

// ───── MQTT → 보드: JSON {"re":42,"ok":1} → "$C,re=42,ok=1*67" ─────
void onMqtt(char *topic, byte *payload, unsigned int len) {
  if (strncmp(topic, BASE, strlen(BASE)) != 0) return;
  const char *leaf = topic + strlen(BASE);
  char type = !strcmp(leaf, "cmd") ? 'C' : !strcmp(leaf, "state") ? 'S' : !strcmp(leaf, "test") ? 'X' : 0;
  if (!type) return;

  JsonDocument doc;                           // payload는 0으로 끝나지 않는다 → 길이를 꼭 넘긴다
  DeserializationError err = deserializeJson(doc, (const char *)payload, len);
  JsonObject obj = doc.as<JsonObject>();
  if (err || obj.isNull()) { DBG("# JSON 오류(%s): %s\n", leaf, err.c_str()); return; }

  char body[LINE_CAP - 3];
  size_t n = 0;
  body[n++] = type; body[n] = 0;
  for (JsonPair kv : obj) {                   // 보낸 순서 그대로 나온다
    const char *k = kv.key().c_str();
    if (type == 'S' && !strcmp(k, "body")) continue;      // state의 body는 보드에 안 보낸다
    char val[48];
    JsonVariant v = kv.value();
    if (v.is<JsonObject>() || v.is<JsonArray>()) { DBG("# 중첩 값 거부: %s\n", k); return; }
    size_t vl = v.is<const char *>() ? strlcpy(val, v.as<const char *>(), sizeof val)
                                     : serializeJson(v, val, sizeof val);   // 숫자는 글자로
    if (vl >= sizeof val || strpbrk(k, ",=*$\r\n") || strpbrk(val, ",=*$\r\n")) { DBG("# 값 거부: %s\n", k); return; }
    int w = snprintf(body + n, sizeof body - n, ",%s=%s", k, val);
    if (w < 0 || (size_t)w >= sizeof body - n) { DBG("# 줄 길이 초과\n"); return; }
    n += w;
  }
  sendToBoard(body);
}

// ───── Wi-Fi / MQTT 유지 (loop에서 계속 호출, 기다리지 않음) ─────
void serviceWifi() {
  static uint32_t lastTry = 0;
  static bool was = false;
  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok && !was) DBG("# WiFi 연결 %s\n", WiFi.localIP().toString().c_str());
  if (!ok && was) DBG("# WiFi 끊김\n");
  was = ok;
  if (!ok && millis() - lastTry >= 15000) {   // 15초마다 처음부터 다시 시도
    lastTry = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}

void serviceMqtt() {
  static bool wasUp = false;
  static uint32_t lastTry = 0, lastLink = 0;
  bool up = mqtt.connected();
  if (up) {
    mqtt.loop();                              // 수신 처리 + keepalive. 자주 불러야 한다
  } else if (WiFi.status() == WL_CONNECTED && millis() - lastTry >= 3000) {
    lastTry = millis();
    // 유언(LWT): 내가 말없이 죽으면 브로커가 link=0(retained)을 대신 발행
    if (mqtt.connect("modem-" STATION, BASE "link", 0, true, "0")) {
      mqtt.publish(BASE "link", "1", true);   // retained
      mqtt.subscribe(BASE "cmd");
      mqtt.subscribe(BASE "state");           // retained라 구독 즉시 현재 상태가 온다 → $S가 보드로 간다
      mqtt.subscribe(BASE "test");
      up = true;
      DBG("# MQTT 연결\n");
    } else DBG("# MQTT 실패 rc=%d\n", mqtt.state());
  }
  if (up != wasUp || (LINK_REPEAT_MS && millis() - lastLink >= LINK_REPEAT_MS)) {
    wasUp = up; lastLink = millis();
    sendToBoard(up ? "L,up=1" : "L,up=0");
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(1024);              // begin 전에. MQTT 재접속으로 잠깐 막혀도 안 넘치게
  Serial2.begin(115200, SERIAL_8N1, PIN_RX2, PIN_TX2);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // 절전 끔: 응답 지연 줄임
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(MQTT_BUF);
  mqtt.setKeepAlive(4);                       // 전원이 뽑히면 약 1.5배(6초쯤) 뒤 브로커가 link=0을 낸다
  mqtt.setSocketTimeout(3);                   // 브로커가 없을 때 connect()가 막히는 시간 줄임
  DBG("\n# modem %s 시작 (TEST_MODE=%d)\n", STATION, TEST_MODE);
}

void loop() {
  pollBoard();
  serviceWifi();
  serviceMqtt();
}
