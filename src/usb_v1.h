#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "decode_stream.h"
#include "protocol.h"

namespace usb_v1
{
  constexpr uint8_t VERSION = 1;
  constexpr std::size_t LINE_CAPACITY = 768;

  enum class TxError : uint8_t
  {
    None,
    AuxTimeout,
    UartWrite,
    UartFlush,
  };

  struct TxRecord
  {
    uint32_t board_ms;
    bool ok;
    uint8_t kind;
    uint8_t id;
    uint8_t command;
    std::array<uint8_t, 3> prefix;
    std::array<uint8_t, protocol::UPLINK_FRAME_SIZE> raw;
    TxError error;
  };

  enum class SystemEvent : uint8_t
  {
    Boot,
    Ready,
    QueueOverflow,
    TaskInitFailed,
    TransactionRelease,
  };

  struct SystemRecord
  {
    uint32_t board_ms;
    SystemEvent event;
    uint32_t count;
    uint8_t id;
    bool ok;
    std::array<char, 24> source;
    std::array<char, 24> task;
    std::array<char, 24> error;
  };

  std::size_t formatRxLine(const ReceivedPacket &packet, uint32_t sequence,
                           bool has_previous, uint32_t previous_ms,
                           char *line, std::size_t capacity);
  std::size_t formatFragmentLine(const ReceivedFragment &fragment,
                                 uint32_t sequence, char *line,
                                 std::size_t capacity);
  std::size_t formatTxLine(const TxRecord &record, char *line,
                           std::size_t capacity);
  std::size_t formatSystemLine(const SystemRecord &record, char *line,
                               std::size_t capacity);
  std::size_t formatPrettyLine(const char *text, char *line,
                               std::size_t capacity);

  const char *txErrorName(TxError error);
  const char *systemEventName(SystemEvent event);

#ifdef ARDUINO
  bool initialize();
  bool enqueueRx(const ReceivedPacket &packet);
  bool enqueueFragment(const ReceivedFragment &fragment);
  bool enqueueTx(const TxRecord &record);
  bool enqueueSystem(const SystemRecord &record);
  bool enqueuePretty(const char *text);
  bool enqueuePrettyf(const char *format, ...);
#endif
} // usb_v1名前空間
