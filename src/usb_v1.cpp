#include "usb_v1.h"

#include <cstdarg>
#include <cstdio>

#ifdef ARDUINO
#include <Arduino.h>
#include <atomic>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#endif

namespace usb_v1
{
  namespace
  {
    class LineBuilder
    {
    public:
      LineBuilder(char *line, std::size_t capacity)
          : line_(line), capacity_(capacity), length_(0), valid_(line != nullptr && capacity != 0)
      {
        if (valid_)
        {
          line_[0] = '\0';
        }
      }

      void append(const char *format, ...)
      {
        if (!valid_)
        {
          return;
        }
        va_list args;
        va_start(args, format);
        const int written = std::vsnprintf(
            line_ + length_, capacity_ - length_, format, args);
        va_end(args);
        if (written < 0 || static_cast<std::size_t>(written) >= capacity_ - length_)
        {
          valid_ = false;
          line_[0] = '\0';
          length_ = 0;
          return;
        }
        length_ += static_cast<std::size_t>(written);
      }

      void appendHex(const uint8_t *bytes, std::size_t length)
      {
        for (std::size_t index = 0; index < length; ++index)
        {
          append("%02X", bytes[index]);
        }
      }

      std::size_t length() const { return valid_ ? length_ : 0; }

    private:
      char *line_;
      std::size_t capacity_;
      std::size_t length_;
      bool valid_;
    };

    const char *decodeErrorToken(const ReceivedPacket &packet)
    {
      return packet.valid ? "NONE" : protocol::decodeErrorName(packet.decode_error);
    }
  } // 無名名前空間

  const char *txErrorName(TxError error)
  {
    switch (error)
    {
    case TxError::None:
      return "NONE";
    case TxError::AuxTimeout:
      return "AUX_TIMEOUT";
    case TxError::UartWrite:
      return "UART_WRITE";
    case TxError::UartFlush:
      return "UART_FLUSH";
    }
    return "UART_WRITE";
  }

  const char *systemEventName(SystemEvent event)
  {
    switch (event)
    {
    case SystemEvent::Boot:
      return "BOOT";
    case SystemEvent::Ready:
      return "READY";
    case SystemEvent::QueueOverflow:
      return "QUEUE_OVERFLOW";
    case SystemEvent::TaskInitFailed:
      return "TASK_INIT_FAILED";
    case SystemEvent::TransactionRelease:
      return "TRANSACTION_RELEASE";
    case SystemEvent::UplinkAborted:
      return "UPLINK_ABORTED";
    }
    return "TASK_INIT_FAILED";
  }

  std::size_t formatRxLine(const ReceivedPacket &packet, uint32_t sequence,
                           bool has_previous, uint32_t previous_ms,
                           char *line, std::size_t capacity)
  {
    LineBuilder out(line, capacity);
    out.append("@RX usb_v=1 seq=%lu board_ms=%lu dt_ms=",
               static_cast<unsigned long>(sequence),
               static_cast<unsigned long>(packet.received_at_ms));
    if (has_previous)
    {
      out.append("%lu", static_cast<unsigned long>(packet.received_at_ms - previous_ms));
    }
    else
    {
      out.append("NA");
    }
    out.append(" rssi_present=%u rssi_raw=", packet.rssi_present ? 1U : 0U);
    if (packet.rssi_present)
    {
      out.append("%u rssi_dbm=%d", packet.rssi_raw,
                 static_cast<int>(packet.rssi_raw) - 256);
    }
    else
    {
      out.append("NA rssi_dbm=NA");
    }
    out.append(" valid=%u header=0x%02X len=%u error=%s raw=",
               packet.valid ? 1U : 0U, packet.header,
               packet.application_length, decodeErrorToken(packet));
    out.appendHex(packet.application.data(), packet.application_length);
    return out.length();
  }

  std::size_t formatFragmentLine(const ReceivedFragment &fragment,
                                 uint32_t sequence, char *line,
                                 std::size_t capacity)
  {
    LineBuilder out(line, capacity);
    out.append("@FRAG usb_v=1 seq=%lu board_ms=%lu reason=%s len=%u raw=",
               static_cast<unsigned long>(sequence),
               static_cast<unsigned long>(fragment.received_at_ms),
               fragmentReasonName(fragment.reason), fragment.length);
    out.appendHex(fragment.raw.data(), fragment.length);
    return out.length();
  }

  std::size_t formatTxLine(const TxRecord &record, char *line,
                           std::size_t capacity)
  {
    LineBuilder out(line, capacity);
    out.append("@TX usb_v=1 board_ms=%lu ok=%u kind=%u id=%u command=0x%02X prefix=",
               static_cast<unsigned long>(record.board_ms), record.ok ? 1U : 0U,
               record.kind, record.id, record.command);
    out.appendHex(record.prefix.data(), record.prefix.size());
    out.append(" len=%u raw=", static_cast<unsigned>(record.raw.size()));
    out.appendHex(record.raw.data(), record.raw.size());
    out.append(" error=%s", txErrorName(record.error));
    return out.length();
  }

  std::size_t formatSystemLine(const SystemRecord &record, char *line,
                               std::size_t capacity)
  {
    LineBuilder out(line, capacity);
    out.append("@SYS usb_v=1 board_ms=%lu event=%s",
               static_cast<unsigned long>(record.board_ms),
               systemEventName(record.event));
    switch (record.event)
    {
    case SystemEvent::QueueOverflow:
      out.append(" source=%s count=%lu", record.source.data(),
                 static_cast<unsigned long>(record.count));
      break;
    case SystemEvent::TaskInitFailed:
      out.append(" task=%s error=%s", record.task.data(), record.error.data());
      break;
    case SystemEvent::TransactionRelease:
      out.append(" id=%u ok=%u", record.id, record.ok ? 1U : 0U);
      break;
    case SystemEvent::UplinkAborted:
      out.append(" kind=%u id=%u command=0x%02X error=%s",
                 record.kind, record.id, record.command, record.error.data());
      break;
    case SystemEvent::Boot:
    case SystemEvent::Ready:
      break;
    }
    return out.length();
  }

  std::size_t formatPrettyLine(const char *text, char *line,
                               std::size_t capacity)
  {
    LineBuilder out(line, capacity);
    out.append("# %s", text == nullptr ? "" : text);
    return out.length();
  }

#ifdef ARDUINO
  namespace
  {
    constexpr UBaseType_t OUTPUT_QUEUE_LENGTH = 24;
    constexpr std::size_t PRETTY_CAPACITY = 512;

    enum class RecordKind : uint8_t
    {
      Rx,
      Fragment,
      Tx,
      System,
      Pretty,
    };

    struct OutputRecord
    {
      RecordKind kind;
      ReceivedPacket rx;
      ReceivedFragment fragment;
      TxRecord tx;
      SystemRecord system;
      std::array<char, PRETTY_CAPACITY> pretty;
    };

    QueueHandle_t output_queue = nullptr;
    std::atomic<uint32_t> dropped_records{0};

    void incrementDropped()
    {
      uint32_t current = dropped_records.load(std::memory_order_relaxed);
      while (current != UINT32_MAX &&
             !dropped_records.compare_exchange_weak(
                 current, current + 1, std::memory_order_relaxed))
      {
      }
    }

    bool enqueue(const OutputRecord &record)
    {
      if (output_queue == nullptr || xQueueSend(output_queue, &record, 0) != pdPASS)
      {
        incrementDropped();
        return false;
      }
      return true;
    }

    void writeLine(const char *line, std::size_t length)
    {
      if (length == 0)
      {
        return;
      }
      Serial.write(reinterpret_cast<const uint8_t *>(line), length);
      Serial.write('\n');
    }

    void outputTask(void *)
    {
      uint32_t rx_sequence = 1;
      uint32_t fragment_sequence = 1;
      uint32_t previous_rx_ms = 0;
      bool has_previous_rx = false;
      char line[LINE_CAPACITY] = {};
      for (;;)
      {
        const uint32_t dropped = dropped_records.exchange(0, std::memory_order_relaxed);
        if (dropped != 0)
        {
          SystemRecord overflow{};
          overflow.board_ms = millis();
          overflow.event = SystemEvent::QueueOverflow;
          overflow.count = dropped;
          std::snprintf(overflow.source.data(), overflow.source.size(), "USB_OUTPUT");
          writeLine(line, formatSystemLine(overflow, line, sizeof(line)));
        }

        OutputRecord record{};
        if (xQueueReceive(output_queue, &record, pdMS_TO_TICKS(10)) != pdPASS)
        {
          continue;
        }
        std::size_t length = 0;
        switch (record.kind)
        {
        case RecordKind::Rx:
          length = formatRxLine(record.rx, rx_sequence, has_previous_rx,
                                previous_rx_ms, line, sizeof(line));
          previous_rx_ms = record.rx.received_at_ms;
          has_previous_rx = true;
          ++rx_sequence;
          break;
        case RecordKind::Fragment:
          length = formatFragmentLine(record.fragment, fragment_sequence,
                                      line, sizeof(line));
          ++fragment_sequence;
          break;
        case RecordKind::Tx:
          length = formatTxLine(record.tx, line, sizeof(line));
          break;
        case RecordKind::System:
          length = formatSystemLine(record.system, line, sizeof(line));
          break;
        case RecordKind::Pretty:
          length = formatPrettyLine(record.pretty.data(), line, sizeof(line));
          break;
        }
        writeLine(line, length);
      }
    }
  } // 無名名前空間

  bool initialize()
  {
    output_queue = xQueueCreate(OUTPUT_QUEUE_LENGTH, sizeof(OutputRecord));
    return output_queue != nullptr &&
           xTaskCreateUniversal(
               outputTask, "usb_output_task", 6144, nullptr, 1, nullptr, 0) == pdPASS;
  }

  bool enqueueRx(const ReceivedPacket &packet)
  {
    OutputRecord record{};
    record.kind = RecordKind::Rx;
    record.rx = packet;
    return enqueue(record);
  }

  bool enqueueFragment(const ReceivedFragment &fragment)
  {
    OutputRecord record{};
    record.kind = RecordKind::Fragment;
    record.fragment = fragment;
    return enqueue(record);
  }

  bool enqueueTx(const TxRecord &tx)
  {
    OutputRecord record{};
    record.kind = RecordKind::Tx;
    record.tx = tx;
    return enqueue(record);
  }

  bool enqueueSystem(const SystemRecord &system)
  {
    OutputRecord record{};
    record.kind = RecordKind::System;
    record.system = system;
    return enqueue(record);
  }

  bool enqueuePretty(const char *text)
  {
    OutputRecord record{};
    record.kind = RecordKind::Pretty;
    const int written = std::snprintf(record.pretty.data(), record.pretty.size(),
                                      "%s", text == nullptr ? "" : text);
    if (written < 0 || static_cast<std::size_t>(written) >= record.pretty.size())
    {
      incrementDropped();
      return false;
    }
    return enqueue(record);
  }

  bool enqueuePrettyf(const char *format, ...)
  {
    OutputRecord record{};
    record.kind = RecordKind::Pretty;
    va_list args;
    va_start(args, format);
    const int written = std::vsnprintf(
        record.pretty.data(), record.pretty.size(), format, args);
    va_end(args);
    if (written < 0 || static_cast<std::size_t>(written) >= record.pretty.size())
    {
      incrementDropped();
      return false;
    }
    return enqueue(record);
  }
#endif
} // usb_v1名前空間
