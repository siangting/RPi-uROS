#pragma once
#include <Arduino.h>
#include <IPAddress.h>

struct uxrCustomTransport;

#ifdef __cplusplus
extern "C" {
#endif
bool   micro_ros_transport_open (struct uxrCustomTransport * transport);
bool   micro_ros_transport_close(struct uxrCustomTransport * transport);
size_t micro_ros_transport_write(struct uxrCustomTransport * transport,
                                 const uint8_t * buf, size_t len, uint8_t * errcode);
size_t micro_ros_transport_read (struct uxrCustomTransport * transport,
                                 uint8_t * buf, size_t len, int timeout, uint8_t * errcode);
#ifdef __cplusplus
}
#endif

void microros_set_agent(IPAddress ip, uint16_t port);
