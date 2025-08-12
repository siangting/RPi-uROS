#include "micro_ros_transport.h"
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP  g_udp;
static IPAddress g_agent_ip;
static uint16_t  g_agent_port = 8888;
static uint16_t  g_local_port = 7777;

void microros_set_agent(IPAddress ip, uint16_t port) {
  g_agent_ip   = ip;
  g_agent_port = port;
}

extern "C" {

bool micro_ros_transport_open(struct uxrCustomTransport *) {
  return g_udp.begin(g_local_port);
}

bool micro_ros_transport_close(struct uxrCustomTransport *) {
  g_udp.stop();
  return true;
}

size_t micro_ros_transport_write(struct uxrCustomTransport *,
                                 const uint8_t * buf, size_t len, uint8_t * errcode)
{
  if (g_udp.beginPacket(g_agent_ip, g_agent_port) != 1) {
    if (errcode) *errcode = 1;
    return 0;
  }
  size_t written = g_udp.write(buf, len);
  if (g_udp.endPacket() != 1) {
    if (errcode) *errcode = 2;
    return 0;
  }
  return written;
}

size_t micro_ros_transport_read(struct uxrCustomTransport *,
                                uint8_t * buf, size_t len, int timeout, uint8_t * errcode)
{
  unsigned long start = millis();
  while ((millis() - start) < (unsigned long)timeout) {
    int packetSize = g_udp.parsePacket();
    if (packetSize > 0) {
      int r = g_udp.read(buf, len);
      return (r > 0) ? (size_t)r : 0;
    }
    delay(1);
  }
  if (errcode) *errcode = 0; // timeout
  return 0;
}

} // extern "C"
