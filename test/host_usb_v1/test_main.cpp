#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "decode_stream.h"
#include "usb_v1.h"

namespace
{
  std::map<std::string, std::string> loadVectors(const char *path)
  {
    std::ifstream input(path);
    assert(input.good());
    std::map<std::string, std::string> vectors;
    std::string line;
    while (std::getline(input, line))
    {
      if (line.empty() || line[0] == '#')
      {
        continue;
      }
      const std::size_t first = line.find('|');
      const std::size_t second = line.find('|', first + 1);
      assert(first != std::string::npos && second != std::string::npos);
      vectors.emplace(line.substr(0, first), line.substr(second + 1));
    }
    return vectors;
  }

  std::string field(const std::string &line, const std::string &key)
  {
    const std::string marker = " " + key + '=';
    const std::size_t start = line.find(marker);
    assert(start != std::string::npos);
    const std::size_t value_start = start + marker.size();
    const std::size_t end = line.find(' ', value_start);
    return line.substr(value_start, end - value_start);
  }

  std::vector<uint8_t> hex(const std::string &value)
  {
    assert(value.size() % 2 == 0);
    std::vector<uint8_t> bytes;
    for (std::size_t index = 0; index < value.size(); index += 2)
    {
      bytes.push_back(static_cast<uint8_t>(
          std::stoul(value.substr(index, 2), nullptr, 16)));
    }
    return bytes;
  }

  void assertLineInvariant(const std::string &line)
  {
    assert(line.find('\n') == std::string::npos);
    assert(line.find('\r') == std::string::npos);
    const std::size_t raw_at = line.find(" raw=");
    if (raw_at == std::string::npos)
    {
      return;
    }
    const std::string raw = field(line, "raw");
    for (char value : raw)
    {
      assert((value >= '0' && value <= '9') || (value >= 'A' && value <= 'F'));
    }
    assert(raw.size() / 2 == std::stoul(field(line, "len")));
    if (line.rfind("@RX ", 0) == 0)
    {
      assert(raw.size() >= 2);
      assert(field(line, "header") == "0x" + raw.substr(0, 2));
    }
  }

  std::string formatRx(const ReceivedPacket &packet, uint32_t sequence,
                       bool has_previous, uint32_t previous_ms)
  {
    std::array<char, usb_v1::LINE_CAPACITY> line{};
    const std::size_t length = usb_v1::formatRxLine(
        packet, sequence, has_previous, previous_ms, line.data(), line.size());
    assert(length > 0);
    return std::string(line.data(), length);
  }

  ReceivedPacket decode(const std::string &raw, uint32_t completed_at_ms,
                        bool add_rssi, uint8_t rssi = 172)
  {
    DecodeStream stream(true);
    DecodeEvent event{};
    const auto bytes = hex(raw);
    for (uint8_t byte : bytes)
    {
      assert(!stream.push(byte, completed_at_ms, completed_at_ms * 1000, event));
    }
    if (add_rssi)
    {
      assert(stream.push(rssi, completed_at_ms + 2,
                         completed_at_ms * 1000 + 2000, event));
    }
    else
    {
      assert(stream.pollTimeout(completed_at_ms + 100, 100, event));
    }
    assert(event.kind == DecodeEventKind::Packet);
    assert(event.packet.received_at_ms == completed_at_ms);
    return event.packet;
  }

  void testGoldenRx(const std::map<std::string, std::string> &vectors)
  {
    const std::array<const char *, 9> names = {
        "RX_A0_VALID", "RX_A1_VALID", "RX_A2_VALID", "RX_A3_VALID",
        "RX_A4_VALID", "RX_A5_VALID", "RX_A6_VALID", "RX_B0_VALID",
        "RX_B1_VALID"};
    uint32_t previous_ms = 0;
    for (std::size_t index = 0; index < names.size(); ++index)
    {
      const std::string &expected = vectors.at(names[index]);
      const uint32_t board_ms = 1000 + static_cast<uint32_t>(index) * 500;
      const ReceivedPacket packet = decode(field(expected, "raw"), board_ms, true);
      assert(packet.valid);
      assert(packet.rssi_present && packet.rssi_raw == 172);
      assert(formatRx(packet, static_cast<uint32_t>(index + 1), index != 0,
                      previous_ms) == expected);
      previous_ms = board_ms;
    }

    const std::string &bad = vectors.at("RX_A0_BAD_CHECKSUM");
    const ReceivedPacket invalid = decode(field(bad, "raw"), 5500, true);
    assert(!invalid.valid);
    assert(invalid.decode_error == protocol::DecodeError::ChecksumMismatch);
    assert(formatRx(invalid, 10, true, 5000) == bad);

    const std::string &absent = vectors.at("RX_A0_RSSI_ABSENT");
    const ReceivedPacket without_rssi = decode(field(absent, "raw"), 6000, false);
    assert(without_rssi.valid && !without_rssi.rssi_present);
    assert(formatRx(without_rssi, 11, true, 5500) == absent);
  }

  void testStreamFragments()
  {
    DecodeEvent event{};
    DecodeStream unknown(true);
    assert(!unknown.push(0x7F, 1, 1000, event));
    assert(!unknown.push(0x12, 2, 2000, event));
    assert(unknown.pollTimeout(102, 100, event));
    assert(event.kind == DecodeEventKind::Fragment);
    assert(event.fragment.reason == FragmentReason::UnknownHeader);
    assert(event.fragment.length == 2 && event.fragment.raw[0] == 0x7F);

    DecodeStream timeout(true);
    assert(!timeout.push(0xA0, 10, 10000, event));
    assert(!timeout.push(0x01, 11, 11000, event));
    assert(timeout.pollTimeout(111, 100, event));
    assert(event.fragment.reason == FragmentReason::FrameTimeout);
    assert(event.fragment.length == 2);

    DecodeStream overflow(true);
    for (std::size_t index = 0; index < FRAGMENT_CAPACITY - 1; ++index)
    {
      assert(!overflow.push(0x7F, static_cast<uint32_t>(index), 0, event));
    }
    assert(overflow.push(0x7F, 24, 0, event));
    assert(event.fragment.reason == FragmentReason::FrameOverflow);
    assert(event.fragment.length == FRAGMENT_CAPACITY);

    DecodeStream resync(true);
    assert(!resync.push(0x00, 1, 0, event));
    assert(!resync.push(0xFF, 2, 0, event));
    assert(resync.push(0xB1, 3, 0, event));
    assert(event.fragment.reason == FragmentReason::Resync);
    assert(event.fragment.length == 2);
    assert(!resync.push(0x07, 4, 0, event));
    assert(!resync.push(0xB6, 5, 0, event));
    assert(resync.push(172, 6, 0, event));
    assert(event.kind == DecodeEventKind::Packet && event.packet.valid);
    assert(event.packet.header == 0xB1);
  }

  void testBackToBack(const std::map<std::string, std::string> &vectors)
  {
    DecodeStream stream(true);
    DecodeEvent event{};
    const auto first = hex(field(vectors.at("RX_B1_VALID"), "raw"));
    const auto second = hex(field(vectors.at("RX_A0_VALID"), "raw"));
    for (uint8_t value : first)
    {
      assert(!stream.push(value, 100, 100000, event));
    }
    assert(stream.push(172, 101, 101000, event));
    assert(event.kind == DecodeEventKind::Packet && event.packet.valid);
    assert(event.packet.header == 0xB1);

    // OS/UART chunk境界に依存せず、任意個数ずつ同じbyte streamを渡せる。
    const std::array<std::size_t, 4> chunks = {1, 7, 3, 64};
    std::size_t offset = 0;
    std::size_t chunk_index = 0;
    while (offset < second.size())
    {
      const std::size_t count = std::min(
          chunks[chunk_index++ % chunks.size()], second.size() - offset);
      for (std::size_t index = 0; index < count; ++index)
      {
        assert(!stream.push(second[offset++], 200, 200000, event));
      }
    }
    assert(stream.push(172, 201, 201000, event));
    assert(event.kind == DecodeEventKind::Packet && event.packet.valid);
    assert(event.packet.header == 0xA0);

    DecodeStream delayed(true);
    const auto delayed_raw = hex(field(vectors.at("RX_B1_VALID"), "raw"));
    assert(!delayed.push(delayed_raw[0], 0, 0, event));
    // task再開時にUART buffer済みbyteを先にdrainすればgap超過でも壊さない。
    assert(!delayed.push(delayed_raw[1], 200, 200000, event));
    assert(!delayed.push(delayed_raw[2], 200, 200000, event));
    assert(delayed.push(172, 200, 200000, event));
    assert(event.kind == DecodeEventKind::Packet && event.packet.valid);
  }

  void testFormatters(const std::map<std::string, std::string> &vectors)
  {
    std::array<char, usb_v1::LINE_CAPACITY> line{};
    const std::array<std::pair<const char *, FragmentReason>, 4> fragments = {{
        {"FRAG_UNKNOWN_HEADER", FragmentReason::UnknownHeader},
        {"FRAG_FRAME_TIMEOUT", FragmentReason::FrameTimeout},
        {"FRAG_FRAME_OVERFLOW", FragmentReason::FrameOverflow},
        {"FRAG_RESYNC", FragmentReason::Resync},
    }};
    uint32_t fragment_sequence = 1;
    for (const auto &entry : fragments)
    {
      const std::string &expected = vectors.at(entry.first);
      ReceivedFragment fragment{};
      fragment.received_at_ms = 6000 + fragment_sequence * 100;
      fragment.reason = entry.second;
      const auto raw = hex(field(expected, "raw"));
      fragment.length = static_cast<uint8_t>(raw.size());
      std::copy(raw.begin(), raw.end(), fragment.raw.begin());
      const std::size_t formatted = usb_v1::formatFragmentLine(
          fragment, fragment_sequence, line.data(), line.size());
      assert(std::string(line.data(), formatted) == expected);
      ++fragment_sequence;
    }

    usb_v1::TxRecord tx{};
    tx.board_ms = 6500;
    tx.ok = true;
    tx.kind = 0;
    tx.id = 42;
    tx.command = 0x13;
    tx.prefix = {0x00, 0x00, 0x04};
    const auto tx_bytes = hex(field(vectors.at("TX_SUCCESS"), "raw"));
    std::copy(tx_bytes.begin(), tx_bytes.end(), tx.raw.begin());
    tx.error = usb_v1::TxError::None;
    std::size_t length = usb_v1::formatTxLine(tx, line.data(), line.size());
    assert(std::string(line.data(), length) == vectors.at("TX_SUCCESS"));
    tx.board_ms = 6600;
    tx.ok = false;
    tx.error = usb_v1::TxError::AuxTimeout;
    length = usb_v1::formatTxLine(tx, line.data(), line.size());
    assert(std::string(line.data(), length) == vectors.at("TX_AUX_TIMEOUT"));

    usb_v1::SystemRecord system{};
    system.board_ms = 12;
    system.event = usb_v1::SystemEvent::Boot;
    length = usb_v1::formatSystemLine(system, line.data(), line.size());
    assert(std::string(line.data(), length) == vectors.at("SYS_BOOT"));
    system = {};
    system.board_ms = 3010;
    system.event = usb_v1::SystemEvent::QueueOverflow;
    system.count = 2;
    std::copy_n("USB_OUTPUT", 11, system.source.begin());
    length = usb_v1::formatSystemLine(system, line.data(), line.size());
    assert(std::string(line.data(), length) == vectors.at("SYS_QUEUE_OVERFLOW"));

    system = {};
    system.board_ms = 5010;
    system.event = usb_v1::SystemEvent::TransactionRelease;
    system.id = 42;
    system.ok = true;
    length = usb_v1::formatSystemLine(system, line.data(), line.size());
    assert(std::string(line.data(), length) ==
           "@SYS usb_v=1 board_ms=5010 event=TRANSACTION_RELEASE id=42 ok=1");

    length = usb_v1::formatPrettyLine("commands:", line.data(), line.size());
    assert(std::string(line.data(), length) == "# commands:");
    assert(std::string(line.data(), length).find('\n') == std::string::npos);

    for (const auto &entry : vectors)
    {
      assertLineInvariant(entry.second);
    }
  }

  void testWrap(const std::map<std::string, std::string> &vectors)
  {
    ReceivedPacket before = decode(field(vectors.at("RX_WRAP_BEFORE"), "raw"),
                                   UINT32_MAX - 5, true, 255);
    assert(formatRx(before, UINT32_MAX, false, 0) == vectors.at("RX_WRAP_BEFORE"));
    ReceivedPacket after = decode(field(vectors.at("RX_WRAP_AFTER"), "raw"),
                                  5, true, 255);
    assert(formatRx(after, 0, true, UINT32_MAX - 5) == vectors.at("RX_WRAP_AFTER"));
  }

  void testErrorNames()
  {
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::None)) == "NONE");
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::ChecksumMismatch)) == "CHECKSUM");
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::WrongLength)) == "INVALID_LENGTH");
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::NonZeroPadding)) == "INVALID_PADDING");
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::InvalidField)) == "INVALID_FIELD");
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::InvalidEnum)) == "INVALID_ENUM");
    assert(std::string(protocol::decodeErrorName(protocol::DecodeError::DecodeFailure)) == "DECODE_ERROR");
  }
} // 無名名前空間

int main()
{
  const auto vectors = loadVectors("testdata/99l_usb_v1_vectors.txt");
  testGoldenRx(vectors);
  testStreamFragments();
  testBackToBack(vectors);
  testFormatters(vectors);
  testWrap(vectors);
  testErrorNames();
  return 0;
}
