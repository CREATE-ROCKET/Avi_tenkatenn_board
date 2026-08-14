#include "decode.h"

#include <Arduino.h>
#include <atomic>

#include "uplink_boundary_policy.h"

namespace
{
  constexpr UBaseType_t DECODE_EVENT_QUEUE_LENGTH = 16;
  constexpr uint32_t FRAME_GAP_TIMEOUT_MS = 100;
  QueueHandle_t decode_event_queue = nullptr;
  QueueHandle_t uplink_boundary_queue = nullptr;
  uint32_t uplink_boundary_sequence = 0;
  uint8_t consecutive_periodic = 0;
  uint32_t last_valid_packet_at = 0;
  bool packet_is_active = false;
  std::atomic<uint32_t> packet_drop_count{0};

  bool isPeriodicHeader(protocol::PacketHeader header)
  {
    const uint8_t value = static_cast<uint8_t>(header);
    return value >= static_cast<uint8_t>(protocol::PacketHeader::CommandReceive) &&
           value <= static_cast<uint8_t>(protocol::PacketHeader::Descent);
  }

  void publishUplinkBoundary(const protocol::DecodedPacket &packet,
                             uint32_t received_at_us)
  {
    UplinkBoundary boundary{};
    if (packet.header == protocol::PacketHeader::GroundTimeRequest)
    {
      boundary.kind = UplinkBoundaryKind::GroundTimeRequest;
      boundary.request_id = packet.time_request.request_id;
      consecutive_periodic = uplink_boundary_policy::resetPeriodicStreak();
    }
    else if (isPeriodicHeader(packet.header))
    {
      consecutive_periodic =
          uplink_boundary_policy::advancePeriodicStreak(consecutive_periodic);
      if (!uplink_boundary_policy::periodicModeActive(
              consecutive_periodic, UPLINK_PERIODIC_ACTIVATION_COUNT))
      {
        return;
      }
      boundary.kind = UplinkBoundaryKind::Periodic;
    }
    else
    {
      // A5 Recovery、A6、B0はuplink開始境界には使用しない。
      return;
    }
    ++uplink_boundary_sequence;
    if (uplink_boundary_sequence == 0)
    {
      ++uplink_boundary_sequence;
    }
    boundary.header = static_cast<uint8_t>(packet.header);
    boundary.sequence = uplink_boundary_sequence;
    boundary.received_at_us = received_at_us;
    if (uplink_boundary_queue != nullptr)
    {
      // 未使用の古い境界はlatestへ置換し、backlogを作らない。
      xQueueOverwrite(uplink_boundary_queue, &boundary);
    }
  }

  void publishEvent(const DecodeEvent &event)
  {
    if (event.kind == DecodeEventKind::Packet && event.packet.valid)
    {
      publishUplinkBoundary(event.packet.decoded, event.packet.received_at_us);
      last_valid_packet_at = event.packet.received_at_ms;
      packet_is_active = true;
      digitalWrite(update_led, HIGH);
    }
    if (decode_event_queue == nullptr ||
        xQueueSend(decode_event_queue, &event, 0) != pdPASS)
    {
      packet_drop_count.fetch_add(1, std::memory_order_relaxed);
    }
  }

  void decodeTask(void *)
  {
    DecodeStream stream(LORA_APPEND_RSSI);
    for (;;)
    {
      DecodeEvent event{};
      while (Serial1.available() > 0)
      {
        const int read_value = Serial1.read();
        if (read_value < 0)
        {
          break;
        }
        if (stream.push(static_cast<uint8_t>(read_value), millis(), micros(), event))
        {
          publishEvent(event);
        }
      }
      // taskが遅延しても、既にUART bufferへ届いた続きよりtimeoutを先にしない。
      if (Serial1.available() == 0 &&
          stream.pollTimeout(millis(), FRAME_GAP_TIMEOUT_MS, event))
      {
        publishEvent(event);
      }

      if (packet_is_active && millis() - last_valid_packet_at >= TELEMETRY_TIMEOUT_MS)
      {
        packet_is_active = false;
        if (uplink_boundary_queue != nullptr)
        {
          // 受信断中のschedule変更を推測せず、startup同様fail-closedへ戻す。
          xQueueReset(uplink_boundary_queue);
          consecutive_periodic = uplink_boundary_policy::resetPeriodicStreak();
        }
        digitalWrite(update_led, LOW);
      }
      delay(1);
    }
  }
} // 無名名前空間

bool start_decode_task()
{
  decode_event_queue = xQueueCreate(DECODE_EVENT_QUEUE_LENGTH, sizeof(DecodeEvent));
  uplink_boundary_queue = xQueueCreate(1, sizeof(UplinkBoundary));
  consecutive_periodic = uplink_boundary_policy::resetPeriodicStreak();
  return decode_event_queue != nullptr && uplink_boundary_queue != nullptr &&
         xTaskCreateUniversal(
             decodeTask, "decode_task", 4096, nullptr, 3, nullptr, 0) == pdPASS;
}

bool receive_uplink_boundary(UplinkBoundary &destination, TickType_t timeout)
{
  return uplink_boundary_queue != nullptr &&
         xQueueReceive(uplink_boundary_queue, &destination, timeout) == pdPASS;
}

bool receive_decode_event(DecodeEvent &destination, TickType_t timeout)
{
  return decode_event_queue != nullptr &&
         xQueueReceive(decode_event_queue, &destination, timeout) == pdPASS;
}

uint32_t dropped_packet_count()
{
  return packet_drop_count.load(std::memory_order_relaxed);
}
