#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/empty.h>
#include <rmw_microros/rmw_microros.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

// == LED Pins ===
#define LED1_PIN 2
#define LED2_PIN 3

// === micro-ROS Objects ===
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t heartbeat_sub;
rclc_executor_t executor;
std_msgs__msg__Empty heartbeat_msg;

// === State Variables ===
unsigned long last_heartbeat_time = 0;
const unsigned long HEARTBEAT_TIMEOUT = 3000;
bool agent_alive = false;

// === FreeRTOS Mutex ===
SemaphoreHandle_t led_mutex;

// === callback ===
void heartbeat_callback(const void *msgin)
{
  (void)msgin;
  unsigned long now = millis();

  // Enter critical section
  if (xSemaphoreTake(led_mutex, portMAX_DELAY) == pdTRUE) {
    last_heartbeat_time = now;
    agent_alive = true;
    xSemaphoreGive(led_mutex);
  }

  Serial.println("💓 Received heartbeat");
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  set_microros_serial_transports(Serial);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);

  // Create mutex for LED and state protection
  led_mutex = xSemaphoreCreateMutex();

  while (RMW_RET_OK != rmw_uros_ping_agent(100, 10)) {
    Serial.println("Waiting for micro-ROS Agent...");
    delay(100);
  }

  Serial.println("Agent ready, initializing...");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pico_heartbeat_monitor", "", &support);

  rclc_subscription_init_default(
      &heartbeat_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
      "/heartbeat");

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &heartbeat_sub, &heartbeat_msg, heartbeat_callback, ON_NEW_DATA);

  Serial.println("Setup complete. Waiting for heartbeat...");
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  unsigned long now = millis();
  bool alive_copy;

  // Copy shared variables safely (critical section)
  if (xSemaphoreTake(led_mutex, portMAX_DELAY) == pdTRUE) {
    if (agent_alive && (now - last_heartbeat_time > HEARTBEAT_TIMEOUT)) {
      agent_alive = false;
      Serial.println("💀 Heartbeat lost!");
    }
    alive_copy = agent_alive;
    xSemaphoreGive(led_mutex);
  }

  // Update LEDs based on current state (outside critical section)
  if (alive_copy) {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
  } else {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
  }

  vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay
}
