#include "decode_stream.h"

#include <algorithm>

DecodeStream::DecodeStream(bool append_rssi)
    : append_rssi_(append_rssi), state_(State::Header), frame_{},
      frame_length_(0), expected_length_(0), last_byte_at_ms_(0),
      pending_packet_{} {}

void DecodeStream::clear()
{
  state_ = State::Header;
  frame_length_ = 0;
  expected_length_ = 0;
}

void DecodeStream::start(uint8_t header, uint32_t now_ms)
{
  frame_[0] = header;
  frame_length_ = 1;
  expected_length_ = protocol::applicationPacketLength(header);
  last_byte_at_ms_ = now_ms;
  state_ = State::Application;
}

void DecodeStream::startUnknown(uint8_t value, uint32_t now_ms)
{
  frame_[0] = value;
  frame_length_ = 1;
  expected_length_ = 0;
  last_byte_at_ms_ = now_ms;
  state_ = State::Unknown;
}

bool DecodeStream::emitPacket(bool rssi_present, uint8_t rssi_raw,
                              DecodeEvent &event)
{
  pending_packet_.rssi_present = rssi_present;
  pending_packet_.rssi_raw = rssi_raw;
  event = {};
  event.kind = DecodeEventKind::Packet;
  event.packet = pending_packet_;
  clear();
  return true;
}

bool DecodeStream::emitFragment(uint32_t now_ms, FragmentReason reason,
                                DecodeEvent &event)
{
  event = {};
  event.kind = DecodeEventKind::Fragment;
  event.fragment.received_at_ms = now_ms;
  event.fragment.reason = reason;
  event.fragment.length = static_cast<uint8_t>(frame_length_);
  std::copy_n(frame_.begin(), frame_length_, event.fragment.raw.begin());
  clear();
  return true;
}

bool DecodeStream::push(uint8_t value, uint32_t now_ms, uint32_t now_us,
                        DecodeEvent &event)
{
  if (state_ == State::Header)
  {
    if (!protocol::isKnownPacketHeader(value))
    {
      startUnknown(value, now_ms);
      return false;
    }
    start(value, now_ms);
    return false;
  }

  last_byte_at_ms_ = now_ms;
  if (state_ == State::Unknown)
  {
    if (protocol::isKnownPacketHeader(value))
    {
      const bool emitted = emitFragment(now_ms, FragmentReason::Resync, event);
      start(value, now_ms);
      return emitted;
    }
    frame_[frame_length_++] = value;
    if (frame_length_ == frame_.size())
    {
      return emitFragment(now_ms, FragmentReason::FrameOverflow, event);
    }
    return false;
  }
  if (state_ == State::Rssi)
  {
    return emitPacket(true, value, event);
  }

  if (frame_length_ >= frame_.size())
  {
    const bool emitted = emitFragment(now_ms, FragmentReason::FrameOverflow, event);
    if (protocol::isKnownPacketHeader(value))
    {
      start(value, now_ms);
    }
    return emitted;
  }

  frame_[frame_length_++] = value;
  if (frame_length_ != expected_length_)
  {
    return false;
  }

  pending_packet_ = {};
  pending_packet_.received_at_ms = now_ms;
  pending_packet_.received_at_us = now_us;
  pending_packet_.header = frame_[0];
  pending_packet_.application_length = static_cast<uint8_t>(frame_length_);
  std::copy_n(frame_.begin(), frame_length_, pending_packet_.application.begin());
  pending_packet_.decode_error = protocol::DecodeError::None;
  pending_packet_.valid = protocol::decodeApplicationFrame(
      frame_.data(), frame_length_, pending_packet_.decoded,
      pending_packet_.decode_error);
  if (append_rssi_)
  {
    state_ = State::Rssi;
    return false;
  }
  return emitPacket(false, 0, event);
}

bool DecodeStream::pollTimeout(uint32_t now_ms, uint32_t timeout_ms,
                               DecodeEvent &event)
{
  if (state_ == State::Header || now_ms - last_byte_at_ms_ < timeout_ms)
  {
    return false;
  }
  if (state_ == State::Rssi)
  {
    return emitPacket(false, 0, event);
  }
  if (state_ == State::Unknown)
  {
    return emitFragment(now_ms, FragmentReason::UnknownHeader, event);
  }
  return emitFragment(now_ms, FragmentReason::FrameTimeout, event);
}

const char *fragmentReasonName(FragmentReason reason)
{
  switch (reason)
  {
  case FragmentReason::UnknownHeader:
    return "UNKNOWN_HEADER";
  case FragmentReason::FrameTimeout:
    return "FRAME_TIMEOUT";
  case FragmentReason::FrameOverflow:
    return "FRAME_OVERFLOW";
  case FragmentReason::Resync:
    return "RESYNC";
  }
  return "RESYNC";
}
