// rfid.ino — 실습 5: RC522 리더기로 키태그를 읽어 우리 줄 형식($T…*XX)으로 찍기 (컴파일 확인 전)
// 대상: Arduino-ESP32 3.3.x + 라이브러리 MFRC522 (GithubCommunity / miguelbalboa, 1.4.x)
// 배선 (공정 ESP32 핀 배정 제안과 같음):  RC522      ESP32
//                                          3.3V   →  3V3   (5V 금지)
//                                          GND    →  GND
//                                          SCK    →  18
//                                          MISO   →  19
//                                          MOSI   →  23
//                                          SDA(SS)→  21
//                                          RST    →  22
//                                          IRQ    →  연결 안 함
// 결과 예: $T{"n":1,"k":"TAG","uid":"04A1B2C3"}*02  ← 이 줄이 실습 7에서 그대로 MQTT로 간다
#include <SPI.h>
#include <MFRC522.h>

#define PIN_SCK  18
#define PIN_MISO 19
#define PIN_MOSI 23
#define PIN_SS   21
#define PIN_RST  22
#define SAME_TAG_MS 2000   // 같은 태그는 2초 안에 다시 찍지 않는다 (리더기 앞에 머무는 동안 한 번만)

MFRC522 rfid(PIN_SS, PIN_RST);
uint8_t n = 0;                      // 신고 번호 1~255 (0은 '시작됨'에만)
char lastUid[21] = "";
uint32_t lastSeen = 0;

uint8_t xorOf(const char *p) { uint8_t c = 0; while (*p) c ^= (uint8_t)*p++; return c; }

void sendLine(const char *body) {   // body = 종류 글자 + 내용
  Serial.printf("$%s*%02X\n", body, xorOf(body));
}

void setup() {
  Serial.begin(115200);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);
  rfid.PCD_Init();
  delay(50);                        // setup 안에서 한 번뿐이라 괜찮다
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.printf("# RC522 버전 0x%02X %s\n", v, (v == 0x91 || v == 0x92 || v == 0x88) ? "(정상)" : "(0x00/0xFF면 배선 확인)");
  sendLine("R{\"n\":0,\"k\":\"BOOT\"}");
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;   // 태그 없으면 바로 돌아간다

  char uid[21] = "";
  for (byte i = 0; i < rfid.uid.size && i < 10; i++) sprintf(uid + i * 2, "%02X", rfid.uid.uidByte[i]);
  rfid.PICC_HaltA();

  if (strcmp(uid, lastUid) == 0 && millis() - lastSeen < SAME_TAG_MS) { lastSeen = millis(); return; }
  strcpy(lastUid, uid);
  lastSeen = millis();

  n = n % 255 + 1;
  char body[80];
  snprintf(body, sizeof body, "T{\"n\":%u,\"k\":\"TAG\",\"uid\":\"%s\"}", n, uid);
  sendLine(body);
}
