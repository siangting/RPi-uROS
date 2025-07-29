#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#include <rclc/publisher.h>
#include <rclc/node.h>
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

// rclc 物件
rcl_allocator_t   allocator;
rclc_support_t    support;
rcl_node_t        node;

// Publisher: 實際回報的角度
rcl_publisher_t         state_pub;
std_msgs__msg__Float32  state_msg;

// Subscriber: 外部下達的目標角度
rcl_subscription_t      cmd_sub;
std_msgs__msg__Float32  cmd_msg;

// Executor + Timer
rclc_executor_t executor;
rcl_timer_t    pub_timer;

// 當前脈衝與角度（在 loop 和 callback 間共用）
volatile uint16_t current_pulse = SERVOMIN;
volatile float    current_angle = 0.0f;

// 當收到新的目標角度，就立即更新伺服
void cmd_callback(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  float target = m->data;
  // 限制在 [0,180]
  if (target < 0.0f) target = 0.0f;
  if (target > 180.0f) target = 180.0f;
  // 計算脈衝
  current_pulse = (uint16_t)((target * (SERVOMAX - SERVOMIN) / 180.0f) + SERVOMIN);
  // 立刻更新所有通道
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    pwm.setPWM(ch, 0, current_pulse);
  }
  // 更新 shared 角度
  current_angle = target;
  Serial.print("CMD → Set angle: ");
  Serial.println(target);
}

// 定時（50 ms）把 current_angle 發佈出去
void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;
  state_msg.data = current_angle;
  rcl_publish(&state_pub, &state_msg, NULL);
  (void)rmw_uros_sync_session(100);
}

void setup() {
  // 1. Serial & transport
  Serial.begin(115200);
  delay(100);
  set_microros_serial_transports(Serial);

  // 2. 等 Agent 上線
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 10)) {
    Serial.println("Waiting for micro-ROS Agent...");
    delay(100);
  }
  Serial.println("Agent ready, initializing...");

  // 3. rclc 初始化
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_node", "", &support);

  // 4. Publisher: /servo_angle_state
  rclc_publisher_init_default(
    &state_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "servo_angle_state"
  );
  state_msg.data = 0.0f;

  // 5. Subscriber: /servo_angle_cmd
  rclc_subscription_init_default(
    &cmd_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "servo_angle_cmd"
  );

  // 6. Timer + Executor
  // 每 50 ms 發一次 state
  rclc_timer_init_default(
    &pub_timer,
    &support,
    RCL_MS_TO_NS(50),
    pub_timer_callback
  );
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &pub_timer);
  rclc_executor_add_subscription(
    &executor,
    &cmd_sub,
    &cmd_msg,
    cmd_callback,
    ON_NEW_DATA
  );

  // 7. I2C & PWM 初始化
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  Serial.println("Setup complete.");
}

void loop() {
  // spin executor 處理 publish/subscription callback
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
