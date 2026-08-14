#include "decode.h"

#include <Arduino.h>
#include <atomic>

#include "uplink_boundary_policy.h"

namespace
{
  constexpr UBaseType_t PACKET_QUEUE_LENGTH = 16;
  QueueHandle_t packet_queue = nullptr;
  QueueHandle_t uplink_boundary_queue = nullptr;
  uint32_t packet_sequence = 0;
  uint32_t uplink_boundary_sequence = 0;
  uint8_t consecutive_periodic = 0;
  uint32_t last_packet_at = 0;
  bool packet_is_active = false;
  std::atomic<uint32_t> packet_drop_count{0};

  bool is_periodic_header(protocol::PacketHeader header)
  {
    const uint8_t value = static_cast<uint8_t>(header);
    return value >= static_cast<uint8_t>(protocol::PacketHeader::CommandReceive) &&
           value <= static_cast<uint8_t>(protocol::PacketHeader::Descent);
  }

  void publish_uplink_boundary(const protocol::DecodedPacket &packet,
                               uint32_t received_at_us)
  {
    UplinkBoundary boundary{};
    if (packet.header == protocol::PacketHeader::GroundTimeRequest)
    {
      boundary.kind = UplinkBoundaryKind::GroundTimeRequest;
      boundary.request_id = packet.time_request.request_id;
      consecutive_periodic = uplink_boundary_policy::resetPeriodicStreak();
    }
    else if (is_periodic_header(packet.header))
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

  void publish_packet(
      const protocol::DecodedPacket &packet,
      uint8_t rssi,
      bool has_rssi)
  {
    const uint32_t received_at = millis();
    const uint32_t received_at_us = micros();
    const bool has_interval = packet_sequence > 0;
    const ReceivedPacket received{
        packet,
        rssi,
        has_rssi,
        received_at,
        received_at_us,
        has_interval ? received_at - last_packet_at : 0,
        has_interval};
    last_packet_at = received_at;
    ++packet_sequence;
    publish_uplink_boundary(packet, received_at_us);

    if (packet_queue == nullptr ||
        xQueueSend(packet_queue, &received, 0) != pdPASS)
    {
      packet_drop_count.fetch_add(1, std::memory_order_relaxed);
    }

    packet_is_active = true;
    digitalWrite(update_led, HIGH);
  }

  void decode_task(void *pvParameters)
  {
    constexpr uint32_t FRAME_GAP_TIMEOUT_MS = 100;
    enum class ReceiveState : uint8_t
    {
      Header,
      ApplicationFrame,
      Rssi,
    };

    ReceiveState state = ReceiveState::Header;
    uint8_t frame[protocol::MAX_APPLICATION_FRAME_SIZE] = {};
    std::size_t frame_length = 0;
    std::size_t expected_length = 0;
    uint32_t last_byte_at = 0;
    protocol::DecodedPacket pending_packet{};

    const auto start_frame = [&](uint8_t value) {
      expected_length = protocol::expectedApplicationLength(value);
      if (expected_length == 0)
      {
        state = ReceiveState::Header;
        frame_length = 0;
        return;
      }
      frame[0] = value;
      frame_length = 1;
      state = ReceiveState::ApplicationFrame;
    };

    while (true)
    {
      if (state != ReceiveState::Header &&
          millis() - last_byte_at >= FRAME_GAP_TIMEOUT_MS)
      {
        if (state == ReceiveState::Rssi)
        {
          // RSSIが欠落しても検証済みapplication packetは保持する。
          publish_packet(pending_packet, 0, false);
        }
        state = ReceiveState::Header;
        frame_length = 0;
      }

      while (Serial1.available() > 0)
      {
        const int read_value = Serial1.read();
        if (read_value < 0)
        {
          break;
        }

        const uint8_t value = static_cast<uint8_t>(read_value);
        last_byte_at = millis();

        if (state == ReceiveState::Header)
        {
          start_frame(value);
          continue;
        }

        if (state == ReceiveState::Rssi)
        {
          publish_packet(pending_packet, value, value != 0);
          state = ReceiveState::Header;
          frame_length = 0;
          continue;
        }

        if (frame_length >= sizeof(frame))
        {
          state = ReceiveState::Header;
          frame_length = 0;
          start_frame(value);
          continue;
        }

        frame[frame_length++] = value;
        if (frame_length != expected_length)
        {
          continue;
        }

        protocol::DecodeError error = protocol::DecodeError::None;
        if (!protocol::decodeApplicationFrame(
                frame, frame_length, pending_packet, error))
        {
          Serial.print("LoRa frame rejected: ");
          Serial.println(protocol::decodeErrorName(error));
          state = ReceiveState::Header;
          frame_length = 0;
          start_frame(value);
          continue;
        }

        if (LORA_APPEND_RSSI)
        {
          state = ReceiveState::Rssi;
        }
        else
        {
          publish_packet(pending_packet, 0, false);
          state = ReceiveState::Header;
          frame_length = 0;
        }
      }

      if (packet_is_active &&
          millis() - last_packet_at >= TELEMETRY_TIMEOUT_MS)
      {
        packet_is_active = false;
        if (uplink_boundary_queue != nullptr)
        {
          // 受信断中のschedule変更を推測せず、startup同様fail-closedへ戻す。
          xQueueReset(uplink_boundary_queue);
          consecutive_periodic =
              uplink_boundary_policy::resetPeriodicStreak();
        }
        digitalWrite(update_led, LOW);
      }

      delay(1);
    }
  }
} // 無名名前空間

bool start_decode_task()
{
  packet_queue = xQueueCreate(PACKET_QUEUE_LENGTH, sizeof(ReceivedPacket));
  uplink_boundary_queue = xQueueCreate(1, sizeof(UplinkBoundary));
  consecutive_periodic = uplink_boundary_policy::resetPeriodicStreak();
  if (packet_queue == nullptr || uplink_boundary_queue == nullptr)
  {
    Serial.println("failed to create packet queue");
    return false;
  }
  return xTaskCreateUniversal(
             decode_task, "decode_task", 4096, nullptr, 3, nullptr, 0) == pdPASS;
}

bool receive_uplink_boundary(UplinkBoundary &destination, TickType_t timeout)
{
  return uplink_boundary_queue != nullptr &&
         xQueueReceive(uplink_boundary_queue, &destination, timeout) == pdPASS;
}

bool receive_packet(ReceivedPacket &destination, TickType_t timeout)
{
  if (packet_queue == nullptr)
  {
    return false;
  }
  return xQueueReceive(packet_queue, &destination, timeout) == pdPASS;
}

uint32_t dropped_packet_count()
{
  return packet_drop_count.load(std::memory_order_relaxed);
}
