#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>

// ===== micro-ROS =====
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

// 你的自訂 UDP 傳輸（需提供 open/close/read/write + microros_set_agent）
#include "micro_ros_transport.h"

// ===================== Wi-Fi / Agent =====================
const char* SSID = "screamlab";
const char* PASS = "s741852scream";
IPAddress   AGENT_IP(192,168,75,41);
const uint16_t AGENT_PORT = 8888;

// ===================== I2C & PWM（PCA9685）=====================
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 0
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 1
#endif

#define SERVOMIN    150
#define SERVOMAX    600
#define SERVO_FREQ  50
#define NUM_SERVOS  16

Adafruit_PWMServoDriver pwm;

// ===================== micro-ROS 物件 =====================
rcl_allocator_t   allocator;
rclc_support_t    support;
rcl_node_t        node;

rcl_publisher_t         state_pub;   // /servo_angle_state
std_msgs__msg__Float32  state_msg;

rcl_subscription_t      cmd_sub;     // /servo_angle_cmd
std_msgs__msg__Float32  cmd_msg;

rclc_executor_t executor;
rcl_timer_t     pub_timer;

volatile uint16_t current_pulse = SERVOMIN;
volatile float    current_angle = 0.0f;

// ===================== 輔助 =====================
#define LOGI(tag, msg)      do { Serial.print("[I] "); Serial.print(tag); Serial.print(": "); Serial.println(msg); } while(0)
#define LOGW(tag, msg)      do { Serial.print("[W] "); Serial.print(tag); Serial.print(": "); Serial.println(msg); } while(0)
#define LOGE(tag, msg)      do { Serial.print("[E] "); Serial.print(tag); Serial.print(": "); Serial.println(msg); } while(0)

static bool entities_created = false;

// 將角度 [0,180] 映射成 PCA9685 的脈衝寬度
static inline uint16_t angle_to_pulse(float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  return (uint16_t)(deg * (SERVOMAX - SERVOMIN) / 180.0f + SERVOMIN);
}

// ========== Subscriber callback：收到角度指令就全通道更新 ==========
void cmd_callback(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  float target = m->data;
  current_pulse = angle_to_pulse(target);
  current_angle = constrain(target, 0.0f, 180.0f);

  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    pwm.setPWM(ch, 0, current_pulse);
  }

  Serial.print("[CMD] Set angle -> ");
  Serial.print(current_angle);
  Serial.print(" deg (pulse=");
  Serial.print(current_pulse);
  Serial.println(")");
}

// ========== Timer callback：每 50ms 發佈目前角度 ==========
void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)timer; (void)last_call_time;
  state_msg.data = current_angle;
  rcl_ret_t rc = rcl_publish(&state_pub, &state_msg, NULL);
  if (rc != RCL_RET_OK) {
    LOGW("publish", String("rc=") + String((int)rc));
  }
  // 同步 session，縮短首次通訊延遲
  (void)rmw_uros_sync_session(100);
}

// ========== 建立 micro-ROS 實體 ==========
bool create_entities() {
  allocator = rcl_get_default_allocator();

  rcl_ret_t rc;

  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) { LOGE("support_init", (int)rc); return false; }

  rc = rclc_node_init_default(&node, "esp32_servo_node", "", &support);
  if (rc != RCL_RET_OK) { LOGE("node_init", (int)rc); return false; }

  rc = rclc_publisher_init_default(
        &state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "servo_angle_state");
  if (rc != RCL_RET_OK) { LOGE("publisher_init", (int)rc); return false; }
  state_msg.data = 0.0f;

  rc = rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "servo_angle_cmd");
  if (rc != RCL_RET_OK) { LOGE("subscription_init", (int)rc); return false; }

  // timer 20Hz（50ms）
  rc = rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(50), pub_timer_callback);
  if (rc != RCL_RET_OK) { LOGE("timer_init", (int)rc); return false; }

  rc = rclc_executor_init(&executor, &support.context, 2, &allocator);
  if (rc != RCL_RET_OK) { LOGE("executor_init", (int)rc); return false; }

  rc = rclc_executor_add_timer(&executor, &pub_timer);
  if (rc != RCL_RET_OK) { LOGE("executor_add_timer", (int)rc); return false; }

  rc = rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, cmd_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK) { LOGE("executor_add_sub", (int)rc); return false; }

  entities_created = true;
  LOGI("micro-ROS", "entities created");
  return true;
}

// ========== 銷毀 micro-ROS 實體（為重連準備） ==========
void destroy_entities() {
  if (!entities_created) return;

  rcl_ret_t rc;
  rc = rcl_publisher_fini(&state_pub, &node);
  rc = rcl_subscription_fini(&cmd_sub, &node);
  rc = rcl_timer_fini(&pub_timer);
  rclc_executor_fini(&executor);
  rc = rcl_node_fini(&node);
  rc = rclc_support_fini(&support);

  entities_created = false;
  LOGW("micro-ROS", "entities destroyed");
}

// ========== 連上 Wi-Fi ==========
void connect_wifi_blocking() {
  if (WiFi.status() == WL_CONNECTED) return;

  LOGI("WiFi", "Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
    if (millis() - t0 > 20000) {   // 20 秒逾時重試
      LOGW("WiFi", "retry...");
      WiFi.disconnect();
      WiFi.begin(SSID, PASS);
      t0 = millis();
    }
  }
  Serial.println();
  LOGI("WiFi", String("OK, IP=") + WiFi.localIP().toString());
}

// ========== 設好自訂 UDP 傳輸並等待 Agent ==========
bool setup_transport_and_wait_agent(uint32_t ping_timeout_ms = 100, int ping_attempts = 10) {
  microros_set_agent(AGENT_IP, AGENT_PORT);
  rmw_uros_set_custom_transport(
    false, nullptr,
    micro_ros_transport_open,
    micro_ros_transport_close,
    micro_ros_transport_write,
    micro_ros_transport_read
  );

  Serial.print("Pinging agent");
  while (rmw_uros_ping_agent(ping_timeout_ms, ping_attempts) != RMW_RET_OK) {
    Serial.print(".");
    delay(200);
    // 若 Wi-Fi 斷線，先把它接回來
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println();
      LOGW("WiFi", "lost, reconnecting...");
      connect_wifi_blocking();
      Serial.print("Pinging agent");
    }
  }
  Serial.println(" OK");
  return true;
}

// ========== PCA9685 初始化 ==========
void init_pwm_driver() {
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // 上電預設到最小脈衝（0 度）
  current_pulse = angle_to_pulse(0.0f);
  current_angle = 0.0f;
  for (uint8_t ch = 0; ch < NUM_SERVOS; ch++) {
    pwm.setPWM(ch, 0, current_pulse);
  }
}

// ===================== Arduino lifecycle =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  connect_wifi_blocking();
  init_pwm_driver();

  setup_transport_and_wait_agent();   // 設定自訂 UDP + 等待 Agent
  create_entities();                  // 建立 micro-ROS 節點/執行器/計時器/通訊
  LOGI("setup", "complete");
}

void loop() {
  // 讓 executor 跑 callback（訂閱/定時發佈）
  if (entities_created) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  }

  // 輕量健康檢查：Wi-Fi & Agent
  static uint32_t last_check = 0;
  if (millis() - last_check > 1000) {
    last_check = millis();

    if (WiFi.status() != WL_CONNECTED) {
      LOGW("WiFi", "disconnected, reconnecting...");
      destroy_entities();            // 先清掉 micro-ROS 實體
      connect_wifi_blocking();       // Wi-Fi 先回來
      setup_transport_and_wait_agent();
      create_entities();             // 重建 micro-ROS
      return;
    }

    if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK) {
      LOGW("Agent", "not reachable, rebuilding entities...");
      destroy_entities();
      setup_transport_and_wait_agent();
      create_entities();
      return;
    }
  }
}
