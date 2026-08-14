#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "protocol.h"

constexpr std::size_t FRAGMENT_CAPACITY = protocol::MAX_APPLICATION_FRAME_SIZE;

enum class DecodeEventKind : uint8_t
{
  Packet,
  Fragment,
};

enum class FragmentReason : uint8_t
{
  UnknownHeader,
  FrameTimeout,
  FrameOverflow,
  Resync,
};

struct ReceivedPacket
{
  uint32_t received_at_ms;
  uint32_t received_at_us;
  uint8_t header;
  uint8_t application_length;
  std::array<uint8_t, protocol::MAX_APPLICATION_FRAME_SIZE> application;
  bool rssi_present;
  uint8_t rssi_raw;
  bool valid;
  protocol::DecodeError decode_error;
  protocol::DecodedPacket decoded;
};

struct ReceivedFragment
{
  uint32_t received_at_ms;
  FragmentReason reason;
  uint8_t length;
  std::array<uint8_t, FRAGMENT_CAPACITY> raw;
};

struct DecodeEvent
{
  DecodeEventKind kind;
  ReceivedPacket packet;
  ReceivedFragment fragment;
};

class DecodeStream
{
public:
  explicit DecodeStream(bool append_rssi);
  bool push(uint8_t value, uint32_t now_ms, uint32_t now_us, DecodeEvent &event);
  bool pollTimeout(uint32_t now_ms, uint32_t timeout_ms, DecodeEvent &event);

private:
  enum class State : uint8_t
  {
    Header,
    Unknown,
    Application,
    Rssi,
  };

  bool emitPacket(bool rssi_present, uint8_t rssi_raw, DecodeEvent &event);
  bool emitFragment(uint32_t now_ms, FragmentReason reason, DecodeEvent &event);
  void start(uint8_t header, uint32_t now_ms);
  void startUnknown(uint8_t value, uint32_t now_ms);
  void clear();

  bool append_rssi_;
  State state_;
  std::array<uint8_t, protocol::MAX_APPLICATION_FRAME_SIZE> frame_;
  std::size_t frame_length_;
  std::size_t expected_length_;
  uint32_t last_byte_at_ms_;
  ReceivedPacket pending_packet_;
};

const char *fragmentReasonName(FragmentReason reason);
