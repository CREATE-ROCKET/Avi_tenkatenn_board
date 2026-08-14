#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "decode_stream.h"

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
bool receive_decode_event(DecodeEvent &destination, TickType_t timeout);
bool receive_uplink_boundary(UplinkBoundary &destination, TickType_t timeout);
uint32_t dropped_packet_count();
