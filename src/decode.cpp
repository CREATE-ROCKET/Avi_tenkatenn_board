#include "decode.h"

#include <Arduino.h>
#include <atomic>

namespace
{
  constexpr UBaseType_t PACKET_QUEUE_LENGTH = 16;
  QueueHandle_t packet_queue = nullptr;
  uint32_t packet_sequence = 0;
  uint32_t last_packet_at = 0;
  bool packet_is_active = false;
  std::atomic<uint32_t> packet_drop_count{0};

  void publish_packet(
      const protocol::DecodedPacket &packet,
      uint8_t rssi,
      bool has_rssi)
  {
    const uint32_t received_at = millis();
    const bool has_interval = packet_sequence > 0;
    const ReceivedPacket received{
        packet,
        rssi,
        has_rssi,
        received_at,
        has_interval ? received_at - last_packet_at : 0,
        has_interval};
    last_packet_at = received_at;
    ++packet_sequence;

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
        digitalWrite(update_led, LOW);
      }

      delay(1);
    }
  }
} // 無名名前空間

bool start_decode_task()
{
  packet_queue = xQueueCreate(PACKET_QUEUE_LENGTH, sizeof(ReceivedPacket));
  if (packet_queue == nullptr)
  {
    Serial.println("failed to create packet queue");
    return false;
  }
  return xTaskCreateUniversal(
             decode_task, "decode_task", 4096, nullptr, 3, nullptr, 0) == pdPASS;
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
