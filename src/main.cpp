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
#define ERROR_THRESHOLD_MS     50000  // 斷線超過 50 秒觸發閃爍

// ================== micro-ROS Objects ==================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t heartbeat_sub;
rclc_executor_t executor;
std_msgs__msg__Empty heartbeat_msg;

// ================== State Variables ==================
bool entities_created = false;
unsigned long disconnect_start_time = 0; 
bool is_disconnect_timer_running = false;

// ================== Entities Management ==================

bool create_entities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "pico_heartbeat_monitor", "", &support) != RCL_RET_OK) return false;
    
    if (rclc_subscription_init_default(
        &heartbeat_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/heartbeat"
    ) != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    
    rclc_executor_add_subscription(&executor, &heartbeat_sub, &heartbeat_msg, 
        [](const void *msgin){ Serial.println("💓 Heartbeat OK"); }, ON_NEW_DATA);

    return true;
}

void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rclc_executor_fini(&executor);
    rcl_subscription_fini(&heartbeat_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ================== Setup ==================
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);

    // 初始開機狀態：紅燈亮，其餘熄滅
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(ERROR_LED_PIN, HIGH); 
}

// ================== Loop ==================
void loop() {
    // --- 1. 檢查 Agent 連線狀態 ---
    bool ping_success = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);

    if (!ping_success) {
        // 如果剛開始斷線，記錄時間並清理 Entities
        if (!is_disconnect_timer_running) {
            disconnect_start_time = millis();
            is_disconnect_timer_running = true;
            if (entities_created) {
                destroy_entities();
                entities_created = false;
            }
        }

        // 判斷是否已經斷線超過 10 秒
        if (millis() - disconnect_start_time > ERROR_THRESHOLD_MS) {
            // 🚨 進入你要求的錯誤閃爍模式
            Serial.println("❌ FATAL ERROR: Agent offline > 10s");
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, LOW);

            digitalWrite(ERROR_LED_PIN, LOW);   // 亮
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(ERROR_LED_PIN, HIGH);  // 滅
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // 斷線 10 秒內：維持紅燈
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        return; 
    }

    // --- 2. 連線恢復處理 ---
    if (is_disconnect_timer_running) {
        is_disconnect_timer_running = false;
        Serial.println("🔄 Agent back online!");
    }
    
    digitalWrite(RED_LED_PIN, LOW);    // 關閉紅燈
    digitalWrite(GREEN_LED_PIN, HIGH); // 點亮綠燈

    if (!entities_created) {
        if (create_entities()) {
            entities_created = true;
        } else {
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(500));
            return;
        }
    }

    // --- 3. 正常運行 ---
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
        entities_created = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}