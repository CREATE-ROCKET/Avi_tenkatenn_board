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
  uint32_t received_at_us;
  uint32_t receive_interval_ms;
  bool has_receive_interval;
};

enum class UplinkBoundaryKind : uint8_t
{
  Periodic,
  GroundTimeRequest,
};

struct UplinkBoundary
{
  UplinkBoundaryKind kind;
  uint8_t header;
  uint8_t request_id;
  uint32_t sequence;
  uint32_t received_at_us;
};

bool start_decode_task();
bool receive_packet(ReceivedPacket &destination, TickType_t timeout);
bool receive_uplink_boundary(UplinkBoundary &destination, TickType_t timeout);
uint32_t dropped_packet_count();
