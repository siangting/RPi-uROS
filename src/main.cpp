#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>

#include "micro_ros_transport.h"   // 你自訂的 UDP 傳輸（open/close/read/write + microros_set_agent）

// ===== Wi-Fi 與 Agent =====
const char* SSID = "screamlab";
const char* PASS = "s741852scream";
IPAddress AGENT_IP(192,168,75,41);
const uint16_t AGENT_PORT = 8888;

rclc_support_t support;
rcl_node_t node;
rcl_publisher_t pub;
std_msgs__msg__Float32 msg;
rcl_allocator_t allocator;

static void die_if_fail(rcl_ret_t rc, const char* where) {
  if (rc != RCL_RET_OK) {
    Serial.print("[ERR] "); Serial.print(where);
    Serial.print(" -> "); Serial.println((int)rc);
    for(;;){ delay(1000); } // 停在這裡，方便你看到錯誤點
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1) 連 Wi-Fi
  Serial.println("Connecting Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  Serial.print("\nWi-Fi OK. IP="); Serial.println(WiFi.localIP());

  // 2) 註冊自訂 UDP 傳輸
  microros_set_agent(AGENT_IP, AGENT_PORT);
  rmw_uros_set_custom_transport(
    false, nullptr,
    micro_ros_transport_open,
    micro_ros_transport_close,
    micro_ros_transport_write,
    micro_ros_transport_read
  );

  // 3) 先 ping agent，等能通
  Serial.print("Pinging agent");
  while (rmw_uros_ping_agent(100, 10) != RMW_RET_OK) { delay(200); Serial.print("."); }
  Serial.println(" OK");

  // 4) 建立 support/node/publisher，每步都檢查回傳碼
  allocator = rcl_get_default_allocator();
  die_if_fail(rclc_support_init(&support, 0, NULL, &allocator), "support_init");
  die_if_fail(rclc_node_init_default(&node, "pico2w_node", "", &support), "node_init");
  die_if_fail(
    rclc_publisher_init_default(
      &pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "pico2w_float"),
    "publisher_init"
  );

  Serial.println("micro-ROS publisher ready.");
}

void loop() {
  static float v = 0.0f;
  msg.data = v++;

  rcl_ret_t rc = rcl_publish(&pub, &msg, NULL);
  if (rc != RCL_RET_OK) {
    Serial.print("[WARN] publish rc="); Serial.println((int)rc);
  } else {
    Serial.print("Published: "); Serial.println(msg.data);
  }
  delay(1000);
}
