#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "protocol.h"

struct ReceivedPacket
{
  protocol::DecodedPacket packet;
  uint8_t rssi;
  bool has_rssi;
  uint32_t received_at_ms;
  uint32_t receive_interval_ms;
  bool has_receive_interval;
};

bool start_decode_task();
bool receive_packet(ReceivedPacket &destination, TickType_t timeout);
uint32_t dropped_packet_count();
