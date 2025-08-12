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
static volatile bool agent_connected = false; // 即時 ping 結果
static volatile bool agent_active = false;    // 已進入穩定連線狀態

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

// ===== 新增：宣告 =====
static bool microros_create_entities(void);
static void microros_destroy_entities(void);

// ===== 新增：建立 micro-ROS entities =====
static bool microros_create_entities(void)
{
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;

  if (rclc_node_init_default(&node, "pico2w_servo_node", "", &support) != RCL_RET_OK) {
    rclc_support_fini(&support);
    return false;
  }

  if (rclc_publisher_init_default(
        &state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "servo_angle_state") != RCL_RET_OK) {
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  if (rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "servo_angle_cmd") != RCL_RET_OK) {
    rcl_publisher_fini(&state_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  const unsigned timer_period_ms = 50;
  if (rclc_timer_init_default(&pub_timer, &support,
        RCL_MS_TO_NS(timer_period_ms), pub_timer_callback) != RCL_RET_OK) {
    rcl_subscription_fini(&cmd_sub, &node);
    rcl_publisher_fini(&state_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    rcl_timer_fini(&pub_timer);
    rcl_subscription_fini(&cmd_sub, &node);
    rcl_publisher_fini(&state_pub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    return false;
  }

  // 確保 cmd_msg 有可寫入空間
  memset(&cmd_msg, 0, sizeof(cmd_msg));
  rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &sub_cmd_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &pub_timer);

  return true;
}

// ===== 新增：銷毀 micro-ROS entities（供重連前清理） =====
static void microros_destroy_entities(void)
{
  // 依相反順序釋放
  rclc_executor_fini(&executor);
  rcl_timer_fini(&pub_timer);
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_publisher_fini(&state_pub, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// ===== 修改：單一 micro-ROS 任務（加入可斷線重連） =====
static void task_micro_ros(void *arg)
{
  (void)arg;

#if USE_SERIAL_TRANSPORT
  Serial.begin(115200);
  delay(100);
  set_microros_serial_transports(Serial);
#else
  // 範例：UDP
  // IPAddress agent_ip(192,168,1,100);
  // size_t agent_port = 8888;
  // set_microros_udp_transports(agent_ip, agent_port);
#endif

  for (;;) {
      agent_connected = false;
      agent_active = false;

      // 等待 Agent 上線
      while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) {
        vTaskDelay(pdMS_TO_TICKS(100));
      }

      if (!microros_create_entities()) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }

      // 已經建立 entity，但還要等到進入 loop 才算 active
      agent_connected = true;
      agent_active = true;  // 在這裡設為 true，表示進入正常 spin loop

      TickType_t lastPingCheck = xTaskGetTickCount();
      bool lost = false;

      while (!lost) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        vTaskDelay(pdMS_TO_TICKS(1));

        if (xTaskGetTickCount() - lastPingCheck >= pdMS_TO_TICKS(1000)) {
          lastPingCheck = xTaskGetTickCount();
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK &&
              rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            lost = true;
          } else {
            agent_connected = true; // ping 成功
          }
        }
      }

      // 斷線
      agent_connected = false;
      agent_active = false;
      microros_destroy_entities();
      vTaskDelay(pdMS_TO_TICKS(200));
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
      if (agent_active) {
        digitalWrite(LED_PIN, HIGH); // 連上且沒 lost
        vTaskDelay(pdMS_TO_TICKS(200));
      } else {
        level = !level; // 閃爍
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
