#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== micro-ROS (Arduino/PlatformIO) =====
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <rmw_microros/rmw_microros.h>

// ===== FreeRTOS (Arduino library on RP2040) =====
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

// ===================== 可調整參數 =====================
// Raspberry Pi Pico / Pico 2 W 預設 I2C 腳位：SDA=4, SCL=5
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 0
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 1
#endif
#define SERVO_FREQ      50          // 50 Hz for analog servos
#define SERVO_MIN_PULSE 150         // 0° (PCA9685 12-bit)
#define SERVO_MAX_PULSE 600         // 180°
#define NUM_SERVOS      6           // 同步多通道

// micro-ROS 連線：預設用 USB CDC Serial
#define USE_SERIAL_TRANSPORT 1

// Task 設定（RP2040 無多核心 pin-to-core API，使用 xTaskCreate）
#define STACK_UROS   (6 * 1024)
#define STACK_SERVO  (3 * 1024)
#define STACK_LED    (2 * 1024)
#define PRIO_UROS    3
#define PRIO_SERVO   2
#define PRIO_LED     1

// LED（Pico / Pico W / Pico 2 W 在 Arduino Core 下通常為 LED_BUILTIN）
#ifndef LED_PIN
#define LED_PIN LED_BUILTIN
#endif

// ===================== 全域物件 =====================
static Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// micro-ROS
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_publisher_t state_pub;
static rcl_subscription_t cmd_sub;
static rcl_timer_t pub_timer;
static rclc_executor_t executor;
static std_msgs__msg__Float32 state_msg;
static std_msgs__msg__Float32 cmd_msg; // <-- Add this global variable above

// FreeRTOS 同步
static QueueHandle_t angle_q;        // 接收要設定的角度（度）
static SemaphoreHandle_t state_mtx;  // 保護 current_angle
static volatile float current_angle_deg = 0.0f;  // 最新角度（由 SERVO 任務寫入）
static volatile bool agent_connected = false;    // micro-ROS Agent 連線狀態

// ===================== 工具函式 =====================
static inline uint16_t angle_to_pulse(float deg)
{
  if (deg < 0.0f) deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  const float k = (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180.0f;
  return (uint16_t)(SERVO_MIN_PULSE + deg * k);
}

// ===================== micro-ROS Callback =====================
static void sub_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;
  float target_deg = m->data;
  // 將目標角度丟給 SERVO 任務處理硬體
  xQueueSend(angle_q, &target_deg, 0);
}

static void pub_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)timer; (void)last_call_time;
  float copy;
  if (xSemaphoreTake(state_mtx, pdMS_TO_TICKS(2)) == pdTRUE) {
    copy = current_angle_deg;
    xSemaphoreGive(state_mtx);
  } else {
    return; // 略過一次
  }
  state_msg.data = copy;
  rcl_ret_t rc = rcl_publish(&state_pub, &state_msg, NULL);
  (void)rc; // 如需偵錯可列印 rc
}

// ===================== Tasks =====================
// 單一 micro-ROS 任務（executor + pub/sub + timer 都在這裡）
static void task_micro_ros(void *arg)
{
  (void)arg;
#if USE_SERIAL_TRANSPORT
  Serial.begin(115200);
  delay(100);
  set_microros_serial_transports(Serial);
#else
  // 範例：UDP（請依需求改 IP/PORT）
  // IPAddress agent_ip(192,168,1,100);
  // size_t agent_port = 8888;
  // set_microros_udp_transports(agent_ip, agent_port);
#endif

  // 等待 Agent 就緒（LED 任務會依據 agent_connected 閃爍/常亮）
  agent_connected = false;
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 10)) {
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  agent_connected = true;

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico2w_servo_node", "", &support);

  // Publisher: 回報目前角度
  rclc_publisher_init_default(
      &state_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "servo_angle_state");

  // Subscriber: 設定目標角度
  rclc_subscription_init_default(
      &cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "servo_angle_cmd");

  // Timer：固定頻率回報角度（20 Hz）
  const unsigned timer_period_ms = 50;
  rclc_timer_init_default(&pub_timer, &support, RCL_MS_TO_NS(timer_period_ms), pub_timer_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &sub_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &pub_timer);

  // 進入 spin 迴圈（同時每 1 秒偵測 Agent 是否仍在線）
  TickType_t lastPingCheck = xTaskGetTickCount();
  for (;;) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    vTaskDelay(pdMS_TO_TICKS(1));

    if (xTaskGetTickCount() - lastPingCheck >= pdMS_TO_TICKS(1000)) {
      lastPingCheck = xTaskGetTickCount();
      // timeout 0.1s、重試 1 次的輕量 ping
      agent_connected = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);
    }
  }

  vTaskDelete(NULL);
}

// 硬體任務：非 micro-ROS（保留，提高穩定性；只存在一個 micro-ROS 任務）
static void task_servo(void *arg)
{
  (void)arg;
  // I2C + PCA9685 初始化（Pico 2 W）
  Wire.setSDA(I2C_SDA_PIN);
  Wire.setSCL(I2C_SCL_PIN);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // 初始角度
  float target = 0.0f;
  uint16_t pulse = angle_to_pulse(target);
  for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) pwm.setPWM(ch, 0, pulse);
  if (xSemaphoreTake(state_mtx, portMAX_DELAY) == pdTRUE) {
    current_angle_deg = target;
    xSemaphoreGive(state_mtx);
  }

  for (;;) {
    // 等待新的目標角度（若 50ms 沒消息就略過）
    if (xQueueReceive(angle_q, &target, pdMS_TO_TICKS(50)) == pdTRUE) {
      pulse = angle_to_pulse(target);
      for (uint8_t ch = 0; ch < NUM_SERVOS; ++ch) pwm.setPWM(ch, 0, pulse);
      if (xSemaphoreTake(state_mtx, pdMS_TO_TICKS(2)) == pdTRUE) {
        current_angle_deg = target;
        xSemaphoreGive(state_mtx);
      }
    }
  }
}

// LED 任務：未連線 -> 閃爍；已連線 -> 常亮
static void task_led(void *arg)
{
  (void)arg;
  pinMode(LED_PIN, OUTPUT);
  bool level = false;
  for (;;) {
    if (agent_connected) {
      digitalWrite(LED_PIN, HIGH);   // 連上：常亮
      vTaskDelay(pdMS_TO_TICKS(200));
    } else {
      level = !level;                // 未連上：閃爍
      digitalWrite(LED_PIN, level ? HIGH : LOW);
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

// ===================== Arduino 進入點 =====================
void setup()
{
  Serial.begin(115200);
  delay(200);

  angle_q = xQueueCreate(8, sizeof(float));
  state_mtx = xSemaphoreCreateMutex();

  // 啟動任務（只建立一個 micro-ROS 任務 + 硬體任務 + LED 任務）
  xTaskCreate(task_micro_ros, "uros",  STACK_UROS / sizeof(StackType_t),  NULL, PRIO_UROS,  NULL);
  xTaskCreate(task_servo,     "servo", STACK_SERVO / sizeof(StackType_t), NULL, PRIO_SERVO, NULL);
  xTaskCreate(task_led,       "led",   STACK_LED   / sizeof(StackType_t), NULL, PRIO_LED,   NULL);
}

void loop()
{
  // 主要邏輯都在 FreeRTOS 任務中
  vTaskDelay(pdMS_TO_TICKS(1000));
}
