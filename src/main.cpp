#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/empty.h>
#include <rmw_microros/rmw_microros.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

// ================== Pin Definitions ==================
#define GREEN_LED_PIN 12
#define RED_LED_PIN   10
#define ERROR_LED_PIN 15

// ================== Timing ==================
#define HEARTBEAT_TIMEOUT_MS    5000
#define AGENT_WAIT_TIMEOUT_MS   10000

// ================== micro-ROS Objects ==================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t heartbeat_sub;
rclc_executor_t executor;
std_msgs__msg__Empty heartbeat_msg;

// ================== State Variables ==================
unsigned long last_heartbeat_time = 0;
bool agent_alive = false;
bool fatal_error = false;   // ⭐ 關鍵：runtime 致命錯誤

// ================== FreeRTOS ==================
SemaphoreHandle_t state_mutex;

// ================== Heartbeat Callback ==================
void heartbeat_callback(const void *msgin)
{
  (void)msgin;
  unsigned long now = millis();

  if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
    last_heartbeat_time = now;
    agent_alive = true;
    xSemaphoreGive(state_mutex);
  }

  Serial.println("💓 Received heartbeat");
}

// ================== Setup ==================
void setup()
{
  Serial.begin(115200);
  delay(100);

  set_microros_serial_transports(Serial);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, HIGH);
  digitalWrite(ERROR_LED_PIN, HIGH);

  state_mutex = xSemaphoreCreateMutex();

  // -------- Agent Wait with Timeout --------
  Serial.println("🔍 Waiting for micro-ROS Agent...");

  unsigned long start_time = millis();
  bool agent_connected = false;

  while (millis() - start_time < AGENT_WAIT_TIMEOUT_MS) {
    if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
      agent_connected = true;
      break;
    }
    delay(100);
  }

  // -------- Fatal: Agent never connected --------
  if (!agent_connected) {
    Serial.println("❌ Agent not found at boot. Entering ERROR state.");
    fatal_error = true;
  }

  if (fatal_error) {
    return;   // ⭐ 直接進 ERROR loop（在 loop() 處理）
  }

  // -------- micro-ROS Init --------
  Serial.println("✅ Agent connected. Initializing micro-ROS...");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_heartbeat_monitor", "", &support);

  rclc_subscription_init_default(
    &heartbeat_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "/heartbeat"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &heartbeat_sub,
    &heartbeat_msg,
    heartbeat_callback,
    ON_NEW_DATA
  );

  Serial.println("🚀 Setup complete. Waiting for heartbeat...");
}

// ================== Loop ==================
void loop()
{
  // -------- ERROR State (Fatal) --------
  if (fatal_error) {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, LOW);

    digitalWrite(ERROR_LED_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));
    digitalWrite(ERROR_LED_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;   // ❌ 不再執行任何正常邏輯
  }

  // -------- Normal Operation --------
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  unsigned long now = millis();
  bool alive_snapshot;

  if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
    if (agent_alive && (now - last_heartbeat_time > HEARTBEAT_TIMEOUT_MS)) {
      agent_alive = false;
      fatal_error = true;   // ⭐ runtime heartbeat failure → fatal
      Serial.println("💀 Heartbeat lost! Entering ERROR state.");
    }
    alive_snapshot = agent_alive;
    xSemaphoreGive(state_mutex);
  }

  // -------- LED State --------
  if (alive_snapshot) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
  } else {
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}
