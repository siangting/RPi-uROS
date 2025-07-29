#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// I2C & PWM
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1
#define SERVOMIN    150
#define SERVOMAX    600
#define SERVO_FREQ  50
#define NUM_SERVOS  16

Adafruit_PWMServoDriver pwm;

// micro-ROS objects
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rcl_publisher_t publisher;
std_msgs__msg__Float32 msg;

// ----------------------------------------------------------------------------
// 先 ping Agent、再建構 ROS entities
// ----------------------------------------------------------------------------
void setup() {
  // 1. Serial & Transport
  Serial.begin(115200);
  delay(100);
  set_microros_serial_transports(Serial);

  // 2. 等 micro-ROS Agent 上線
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 10)) {
    Serial.println("Waiting for micro-ROS Agent...");
    delay(100);
  }
  Serial.println("Agent is up! Initializing entities...");

  // 3. rclc 初始化 node + publisher
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_servo_node", "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "servo_angle"
  );
  msg.data = 0.0f;

  // 4. I2C & PWM 初始化
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  Serial.println("Setup complete.");
}

// ----------------------------------------------------------------------------
// 在 loop 裡更新 PWM、計算角度、publish + sync
// ----------------------------------------------------------------------------
void loop() {
  // 0° → 180° → 0° 週期
  static int dir = +1;
  static uint16_t pulse = SERVOMIN;

  // 計算角度
  float angle = (pulse - SERVOMIN) * 180.0f / (SERVOMAX - SERVOMIN);
  // PWM 輸出給所有通道
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    pwm.setPWM(ch, 0, pulse);
  }

  // 發佈角度到 /servo_angle
  msg.data = angle;
  rcl_publish(&publisher, &msg, NULL);
  // 立刻 flush 到 Agent
  (void)rmw_uros_sync_session(1000);

  // UART debug
  Serial.print("Published angle: ");
  Serial.print(angle);
  Serial.println("°");

  // 逐步改變脈衝
  pulse += dir;
  if (pulse >= SERVOMAX || pulse <= SERVOMIN) {
    dir = -dir;
  }

  delay(10);
}
