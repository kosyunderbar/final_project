// rtos_step1.ino — 실습 10 1단계: FreeRTOS 태스크 두 개 + 큐 두 개 (컴파일 확인 전)
// 대상: Arduino-ESP32 3.3.x. 부품: 내장 LED(IO2), BOOT 버튼(IO0). 선택: IO25 LED + 220Ω
// Wi-Fi는 아직 쓰지 않는다. secrets.h도 필요 없다.
#include <Arduino.h>

#define PIN_LED_CTRL 2     // 설비 태스크만 만진다 (내장 LED)
#define PIN_BTN      0     // 설비 태스크만 만진다 (BOOT 버튼, 눌림 = LOW)
#define PIN_LED_COMM 25    // 통신 태스크만 만진다 (없으면 시리얼 출력으로 확인)

struct Msg { char line[160]; };   // 큐 한 칸 = 한 줄 (교본 5장의 줄 형식, 칩 안이라 검사값 생략)
QueueHandle_t toComm, toCtrl;     // 설비 → 통신 (신고 큐) / 통신 → 설비 (명령 큐)

// ───── 설비 태스크: 코어 1 · 우선순위 3 · 10ms 박자. 절대 기다리지 않는다 ─────
void ctrlTask(void *) {
  pinMode(PIN_LED_CTRL, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  TickType_t t = xTaskGetTickCount();
  uint32_t tick = 0, tEdge = 0, holdUntil = 0, n = 0;
  bool stable = HIGH, last = HIGH;
  for (;;) {
    tick++;                                        // 10ms마다 1씩
    bool now = digitalRead(PIN_BTN);               // 버튼: 30ms(3박자) 같은 값이면 진짜
    if (now != last) { last = now; tEdge = tick; }
    if (now != stable && tick - tEdge >= 3) {
      stable = now;
      if (stable == LOW) {                         // 눌린 순간 한 번
        Msg m;
        n = n % 255 + 1;                           // 신고 번호 1~255
        snprintf(m.line, sizeof m.line, "$R{\"n\":%lu,\"k\":\"BTN\"}", (unsigned long)n);
        xQueueSend(toComm, &m, 0);                 // 기다림 0. 꽉 차면 못 넣는다(1단계는 그냥 둔다)
      }
    }
    Msg in;
    while (xQueueReceive(toCtrl, &in, 0) == pdTRUE) {   // 기다림 0, 있는 만큼만 꺼낸다
      if (in.line[1] == 'C') holdUntil = tick + 50;     // 답이 오면 0.5초 동안 켜 둔다
    }
    bool on = (tick < holdUntil) || ((tick / 20) % 2);  // 평소에는 0.2초마다 뒤집기
    digitalWrite(PIN_LED_CTRL, on);
    vTaskDelayUntil(&t, pdMS_TO_TICKS(10));        // delay() 대신. 10ms 박자를 지킨다
  }
}

// ───── 통신 태스크: 코어 0 · 우선순위 2. Serial도 이 태스크만 쓴다 ─────
void commTask(void *) {
  pinMode(PIN_LED_COMM, OUTPUT);
  bool led = false;
  uint32_t lastBlink = 0;
  for (;;) {
    // 2단계: 여기에 modem.ino의 serviceWifi(); serviceMqtt(); 가 들어간다
    Msg out;
    while (xQueueReceive(toComm, &out, 0) == pdTRUE) {
      Serial.printf("[N] 받음 %s\n", out.line);    // 2단계: 여기서 MQTT로 발행
      int n = 0;
      sscanf(out.line, "$R{\"n\":%d", &n);
      Msg ans;
      snprintf(ans.line, sizeof ans.line, "$C,re=%d,ok=1", n);
      xQueueSend(toCtrl, &ans, 0);                 // 답을 설비 쪽으로 (2단계: cmd 토픽으로 온 답)
    }
    if (millis() - lastBlink >= 1000) {            // 1초마다 뒤집기 (설비 쪽보다 느리게)
      lastBlink = millis();
      led = !led;
      digitalWrite(PIN_LED_COMM, led);
    }
    vTaskDelay(pdMS_TO_TICKS(5));                  // 잠깐 쉬어 다른 태스크(워치독 포함)에 차례를 준다
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n[N] rtos_step1 시작");         // 태스크를 만들기 전이라 섞이지 않는다
  toComm = xQueueCreate(8, sizeof(Msg));           // 8칸 × 160바이트
  toCtrl = xQueueCreate(8, sizeof(Msg));
  xTaskCreatePinnedToCore(ctrlTask, "ctrl", 6144, nullptr, 3, nullptr, 1);   // 스택 6KB, 코어 1
  xTaskCreatePinnedToCore(commTask, "comm", 8192, nullptr, 2, nullptr, 0);   // 스택 8KB, 코어 0
}

void loop() { vTaskDelete(nullptr); }              // loop는 쓸 일이 없어 스스로 지운다
