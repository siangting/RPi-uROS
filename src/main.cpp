#include <Arduino.h>

// ==== RP2040 / Pico 2 W: UART1 腳位（合法組合：5/4, 9/8, 21/20）====

#define RX_PIN 5

#define TX_PIN 4

#define BusSerial Serial2   // 用 UART1

// STBus 協議
#define HDR       0x55
#define CMD_READ  0x1C
#define CMD_MOVE  0x01
#define CMD_LOAD  0x1F

uint8_t chk(const uint8_t* b){
  uint16_t s = 0;
  for (uint8_t i = 2; i < b[3] + 2; i++) s += b[i];
  return ~s;
}

void sendPack(uint8_t id, uint8_t cmd, const uint8_t* p, uint8_t n){
  uint8_t buf[6 + 16];              // 預留空間避免 VLA；n <= 16
  buf[0] = buf[1] = HDR;
  buf[2] = id;
  buf[3] = n + 3;
  buf[4] = cmd;
  for (uint8_t i = 0; i < n; i++) buf[5 + i] = p[i];
  buf[5 + n] = chk(buf);
  BusSerial.write(buf, 6 + n);
}

void scan(){
  Serial.println("scan 1~30...");
  for (uint8_t i = 1; i <= 30; i++) {
    while (BusSerial.available()) BusSerial.read();
    sendPack(i, CMD_READ, nullptr, 0);
    delay(30);
    if (BusSerial.available() >= 8) {
      Serial.printf("  ID=%d OK\n", i);
      while (BusSerial.available()) BusSerial.read();
    }
  }
}

void setup(){
  Serial.begin(115200);
  delay(50);
  // 指定 UART 腳位 → 再 begin
  BusSerial.setRX(5);
  BusSerial.setTX(4);
  BusSerial.begin(115200 , SERIAL_8N1);
  delay(50);

  // 上載扭力
  uint8_t on = 1;
  sendPack(1, CMD_LOAD, &on, 1);
  delay(50);

  // 掃描 + 初次擺動
  scan();
}

void loop(){
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
  Serial.println("In loop");

  uint8_t p0[]  = {0x00,0x00,100,0x00};
  uint8_t p90[] = {0x77,0x01,100,0x00};

  sendPack(1, CMD_MOVE, p0, 4);
  Serial.println("Servo 1 → Move to 0°");
  delay(500);
  sendPack(1, CMD_MOVE, p90, 4);
  Serial.println("Servo 1 → Move to 90°");
  delay(500);
}
