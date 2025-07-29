#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

// Pi Pico → PCA9685 接線
// 3V3   → VCC (邏輯電源 3.3 V)
// VSYS  → V+  (伺服電源 5 V)
// GND   → GND
// GP0   → SDA (I2C0 SDA)
// GP1   → SCL (I2C0 SCL)

#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN    150
#define SERVOMAX    600
#define SERVO_FREQ  50
#define NUM_SERVOS  16

void setup() {
  Serial.begin(115200);

  // 設定 I2C0 用 GP0/GP1
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();

  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}

void loop() {
  // 0°→180°
  for (uint16_t pulse = SERVOMIN; pulse <= SERVOMAX; pulse++) {
    for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
      pwm.setPWM(ch, 0, pulse);
    }
    delay(10);
  }
  delay(500);
  // 180°→0°
  for (uint16_t pulse = SERVOMAX; pulse >= SERVOMIN; pulse--) {
    for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
      pwm.setPWM(ch, 0, pulse);
    }
    delay(10);
  }
  delay(500);
}
