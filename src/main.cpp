#include <Arduino.h>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "config.h"
#include "decode.h"
#include "protocol.h"
#include "uplink_boundary_policy.h"
#include "usb_v1.h"

#ifndef GROUND_LORA_TIMING_DEBUG
#define GROUND_LORA_TIMING_DEBUG 0
#endif

namespace
{
  constexpr UBaseType_t COMMAND_QUEUE_LENGTH = 16;
  constexpr UBaseType_t EMERGENCY_COMMAND_QUEUE_LENGTH = 2;
  constexpr std::size_t CONSOLE_LINE_SIZE = 128;
  constexpr uint8_t COMMAND_RESULT_ACTUATOR_EMERGENCY = 0xF0;
  constexpr uint8_t COMMAND_RESULT_LIFTOFF_EMERGENCY = 0xF1;

  enum class CommandMessageKind : uint8_t
  {
    MissionGeneric,
    ActuatorEmergency,
    LiftoffEmergency,
    ComBoardLocal,
    GroundTimeResponse,
    ReleaseTransaction,
  };

  struct CommandMessage
  {
    CommandMessageKind kind;
    uint8_t command;
    std::array<uint8_t, 6> args;
    uint32_t requested_at_us;
    uint32_t dequeued_at_us;
  };

  struct UplinkTiming
  {
    uint32_t requested_at_us = 0;
    uint32_t dequeued_at_us = 0;
    uint32_t aux_ready_at_us = 0;
    uint32_t write_started_at_us = 0;
    uint32_t write_finished_at_us = 0;
    uint32_t flush_finished_at_us = 0;
    uint32_t aux_low_at_us = 0;
    uint32_t aux_high_at_us = 0;
    uint32_t completed_at_us = 0;
    uint32_t boundary_sequence = 0;
    uint32_t boundary_received_at_us = 0;
    uint32_t boundary_age_us = 0;
    uint32_t boundary_wait_us = 0;
    uint8_t boundary_header = 0;
    UplinkBoundaryKind boundary_kind = UplinkBoundaryKind::Periodic;
    bool boundary_observed = false;
    bool boundary_fallback = false;
    bool aux_low_observed = false;
  };

  struct UplinkWriteResult
  {
    bool ok;
    usb_v1::TxError error;
    uint32_t completed_at_ms;
  };

  QueueHandle_t command_queue = nullptr;
  QueueHandle_t emergency_command_queue = nullptr;
  SemaphoreHandle_t transaction_mutex = nullptr;
  protocol::TransactionTracker transaction_tracker;
  char console_line[CONSOLE_LINE_SIZE] = {};
  std::size_t console_line_length = 0;

  enum class TxOpportunity : uint8_t
  {
    Ready,
    EmergencyPending,
    Timeout,
  };

  bool takeEmergencyCommand(CommandMessage &message);

  const char *boundaryKindName(UplinkBoundaryKind kind)
  {
    return kind == UplinkBoundaryKind::Periodic ? "periodic" : "time_request";
  }

  const char *commandKindName(CommandMessageKind kind)
  {
    switch (kind)
    {
    case CommandMessageKind::MissionGeneric:
      return "mission_generic";
    case CommandMessageKind::ActuatorEmergency:
      return "actuator_emergency";
    case CommandMessageKind::LiftoffEmergency:
      return "liftoff_emergency";
    case CommandMessageKind::ComBoardLocal:
      return "comboard_local";
    case CommandMessageKind::GroundTimeResponse:
      return "ground_time_response";
    case CommandMessageKind::ReleaseTransaction:
      return "release_transaction";
    }
    return "unknown";
  }

  bool reserveTransaction(protocol::UplinkKind kind, uint8_t command,
                          uint8_t &transaction_id)
  {
    xSemaphoreTake(transaction_mutex, portMAX_DELAY);
    const bool reserved = transaction_tracker.reserve(kind, command, transaction_id);
    xSemaphoreGive(transaction_mutex);
    return reserved;
  }

  bool releaseTransaction(uint8_t transaction_id)
  {
    xSemaphoreTake(transaction_mutex, portMAX_DELAY);
    const bool released = transaction_tracker.release(transaction_id);
    xSemaphoreGive(transaction_mutex);
    return released;
  }

  bool markTransactionResult(const protocol::CommandResult &result)
  {
    xSemaphoreTake(transaction_mutex, portMAX_DELAY);
    const bool matched = transaction_tracker.markResult(result);
    xSemaphoreGive(transaction_mutex);
    return matched;
  }

  bool waitAuxHigh(uint32_t timeout_ms)
  {
    const uint32_t started_at = millis();
    while (digitalRead(aux) != HIGH)
    {
      if (millis() - started_at >= timeout_ms)
      {
        return false;
      }
      vTaskDelay(pdMS_TO_TICKS(AUX_POLL_INTERVAL_MS));
    }
    return true;
  }

  TxOpportunity waitForNormalTxOpportunity(CommandMessage &emergency)
  {
    const uint32_t started_at = millis();
    while (true)
    {
      // AUX待機中もEmergency queueを確認し、選択済み通常commandより先に送る。
      if (takeEmergencyCommand(emergency))
      {
        return TxOpportunity::EmergencyPending;
      }
      if (digitalRead(aux) == HIGH)
      {
        // High判定直後に届いたEmergencyも先に処理する。
        if (takeEmergencyCommand(emergency))
        {
          return TxOpportunity::EmergencyPending;
        }
        return TxOpportunity::Ready;
      }
      if (millis() - started_at >= AUX_TIMEOUT_MS)
      {
        return TxOpportunity::Timeout;
      }
      vTaskDelay(pdMS_TO_TICKS(AUX_POLL_INTERVAL_MS));
    }
  }

  bool receiveBoundaryForMessage(const CommandMessage &message,
                                 UplinkBoundary &boundary,
                                 TickType_t timeout)
  {
    if (!receive_uplink_boundary(boundary, timeout))
    {
      return false;
    }
    // 時刻応答だけはB1完了を必須とし、他commandは単一queueの境界を共有する。
    return message.kind != CommandMessageKind::GroundTimeResponse ||
           boundary.kind == UplinkBoundaryKind::GroundTimeRequest;
  }

  TxOpportunity waitForUplinkBoundary(const CommandMessage &message,
                                      CommandMessage &emergency,
                                      UplinkBoundary &boundary,
                                      UplinkTiming &timing,
                                      uint32_t overall_started_at_ms,
                                      uint32_t overall_started_at_us)
  {
    const bool current_is_emergency =
        message.kind == CommandMessageKind::ActuatorEmergency ||
        message.kind == CommandMessageKind::LiftoffEmergency;
    while (true)
    {
      if (uplink_boundary_policy::deadlineExpiredMs(
              millis(), overall_started_at_ms,
              UPLINK_BOUNDARY_TIMEOUT_MS))
      {
        timing.boundary_wait_us = micros() - overall_started_at_us;
        return TxOpportunity::Timeout;
      }
      if (!current_is_emergency && takeEmergencyCommand(emergency))
      {
        return TxOpportunity::EmergencyPending;
      }

      if (receiveBoundaryForMessage(message, boundary, pdMS_TO_TICKS(1)))
      {
        const uint32_t accepted_at_us = micros();
        const uint32_t age_us = uplink_boundary_policy::elapsedUs(
            accepted_at_us, boundary.received_at_us);
        if (!uplink_boundary_policy::isFresh(
                accepted_at_us, boundary.received_at_us,
                UPLINK_BOUNDARY_FRESH_US))
        {
          continue;
        }
        if (!current_is_emergency && takeEmergencyCommand(emergency))
        {
          // 競合なく一境界一送信を守るため、取得済み境界は再投入しない。
          return TxOpportunity::EmergencyPending;
        }
        timing.boundary_observed = true;
        timing.boundary_kind = boundary.kind;
        timing.boundary_header = boundary.header;
        timing.boundary_sequence = boundary.sequence;
        timing.boundary_received_at_us = boundary.received_at_us;
        timing.boundary_age_us = age_us;
        timing.boundary_wait_us = accepted_at_us - overall_started_at_us;
        return TxOpportunity::Ready;
      }
    }
  }

  bool parseUnsigned(const char *text, uint32_t maximum, uint32_t &value)
  {
    if (text == nullptr || *text == '\0' || *text == '-')
    {
      return false;
    }
    errno = 0;
    char *end = nullptr;
    const unsigned long parsed = std::strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > maximum)
    {
      return false;
    }
    value = static_cast<uint32_t>(parsed);
    return true;
  }

  bool enqueueCommand(const CommandMessage &message)
  {
    if (command_queue == nullptr)
    {
      usb_v1::enqueuePretty("command queue full");
      return false;
    }
    if (xQueueSend(command_queue, &message, 0) != pdPASS)
    {
      usb_v1::enqueuePretty("command queue full");
      return false;
    }
    return true;
  }

  bool notifyEmergency(CommandMessageKind kind, uint32_t requested_at_us)
  {
    if (emergency_command_queue == nullptr)
    {
      return false;
    }
    CommandMessage message{};
    message.kind = kind;
    message.requested_at_us = requested_at_us;
    // 同種Emergencyも別entryとして保持し、満杯なら呼出側へ明示的に失敗を返す。
    return xQueueSend(emergency_command_queue, &message, 0) == pdPASS;
  }

  void printUsage()
  {
    usb_v1::enqueuePretty("commands:");
    usb_v1::enqueuePretty("  g <command> [arg0 ... arg5]");
    usb_v1::enqueuePretty("  ae | le");
    usb_v1::enqueuePretty("  local <command> [arg0 ... arg5]");
    usb_v1::enqueuePretty("  time <request_id> <unix_seconds> <milliseconds>");
    usb_v1::enqueuePretty("  release <transaction_id>");
    usb_v1::enqueuePretty("values accept decimal or 0x-prefixed hexadecimal");
  }

  void parseArgs(char *save, std::array<uint8_t, 6> &args, bool &valid)
  {
    args.fill(0);
    for (std::size_t index = 0; index < args.size(); ++index)
    {
      char *token = strtok_r(nullptr, " \t", &save);
      if (token == nullptr)
      {
        return;
      }
      uint32_t value = 0;
      if (!parseUnsigned(token, UINT8_MAX, value))
      {
        valid = false;
        return;
      }
      args[index] = static_cast<uint8_t>(value);
    }
    if (strtok_r(nullptr, " \t", &save) != nullptr)
    {
      valid = false;
    }
  }

  void processConsoleLine(char *line)
  {
    char *save = nullptr;
    char *operation = strtok_r(line, " \t", &save);
    if (operation == nullptr)
    {
      return;
    }
    if (std::strcmp(operation, "help") == 0 || std::strcmp(operation, "?") == 0)
    {
      printUsage();
      return;
    }

    CommandMessage message{};
    message.requested_at_us = micros();
    if (std::strcmp(operation, "ae") == 0 || std::strcmp(operation, "le") == 0)
    {
      if (strtok_r(nullptr, " \t", &save) != nullptr)
      {
        usb_v1::enqueuePretty("unexpected emergency argument");
        return;
      }
      message.kind = std::strcmp(operation, "ae") == 0
                         ? CommandMessageKind::ActuatorEmergency
                         : CommandMessageKind::LiftoffEmergency;
      if (!notifyEmergency(message.kind, message.requested_at_us))
      {
        usb_v1::enqueuePretty("emergency notification failed");
      }
      return;
    }

    if (std::strcmp(operation, "time") == 0)
    {
      char *request_text = strtok_r(nullptr, " \t", &save);
      char *seconds_text = strtok_r(nullptr, " \t", &save);
      char *milliseconds_text = strtok_r(nullptr, " \t", &save);
      uint32_t request = 0;
      uint32_t seconds = 0;
      uint32_t milliseconds = 0;
      if (!parseUnsigned(request_text, UINT8_MAX, request) || request == 0 ||
          !parseUnsigned(seconds_text, UINT32_MAX, seconds) ||
          !parseUnsigned(milliseconds_text, 999, milliseconds) ||
          strtok_r(nullptr, " \t", &save) != nullptr)
      {
        usb_v1::enqueuePretty("usage: time <request_id 1..255> <unix_seconds> <milliseconds 0..999>");
        return;
      }
      message.kind = CommandMessageKind::GroundTimeResponse;
      message.command = static_cast<uint8_t>(request);
      message.args = {
          static_cast<uint8_t>(seconds),
          static_cast<uint8_t>(seconds >> 8U),
          static_cast<uint8_t>(seconds >> 16U),
          static_cast<uint8_t>(seconds >> 24U),
          static_cast<uint8_t>(milliseconds),
          static_cast<uint8_t>(milliseconds >> 8U)};
      enqueueCommand(message);
      return;
    }

    if (std::strcmp(operation, "release") == 0)
    {
      char *transaction_text = strtok_r(nullptr, " \t", &save);
      uint32_t transaction_id = 0;
      if (!parseUnsigned(transaction_text, UINT8_MAX, transaction_id) ||
          transaction_id == 0 || strtok_r(nullptr, " \t", &save) != nullptr)
      {
        usb_v1::enqueuePretty("usage: release <transaction_id 1..255>");
        return;
      }
      message.kind = CommandMessageKind::ReleaseTransaction;
      message.command = static_cast<uint8_t>(transaction_id);
      enqueueCommand(message);
      return;
    }

    const bool generic = std::strcmp(operation, "g") == 0;
    const bool local = std::strcmp(operation, "local") == 0;
    if (!generic && !local)
    {
      usb_v1::enqueuePretty("unknown command; type help");
      return;
    }
    char *command_text = strtok_r(nullptr, " \t", &save);
    uint32_t command = 0;
    bool valid = parseUnsigned(command_text, UINT8_MAX, command);
    parseArgs(save, message.args, valid);
    if (!valid)
    {
      usb_v1::enqueuePretty("invalid command or argument");
      return;
    }
    message.kind = generic ? CommandMessageKind::MissionGeneric
                           : CommandMessageKind::ComBoardLocal;
    message.command = static_cast<uint8_t>(command);
    enqueueCommand(message);
  }

  void readConsole()
  {
    while (Serial.available() > 0)
    {
      const int read_value = Serial.read();
      if (read_value < 0)
      {
        return;
      }
      const char value = static_cast<char>(read_value);
      if (value == '\r' || value == '\n')
      {
        if (console_line_length != 0)
        {
          console_line[console_line_length] = '\0';
          processConsoleLine(console_line);
          console_line_length = 0;
        }
        continue;
      }
      if (!std::isprint(static_cast<unsigned char>(value)))
      {
        continue;
      }
      if (console_line_length + 1 >= sizeof(console_line))
      {
        console_line_length = 0;
        usb_v1::enqueuePretty("console line too long");
        continue;
      }
      console_line[console_line_length++] = value;
    }
  }

  void updateStateLeds(protocol::PacketHeader header)
  {
    const bool liftoff =
        header == protocol::PacketHeader::EngineBurn ||
        header == protocol::PacketHeader::Control ||
        header == protocol::PacketHeader::Descent ||
        header == protocol::PacketHeader::RecoveryBeacon;
    digitalWrite(liftoff_led, liftoff ? HIGH : LOW);
    digitalWrite(control_led,
                 header == protocol::PacketHeader::Control ? HIGH : LOW);
    digitalWrite(top_led,
                 header == protocol::PacketHeader::Descent ||
                         header == protocol::PacketHeader::RecoveryBeacon
                     ? HIGH
                     : LOW);
  }

  void handlePacket(const ReceivedPacket &received)
  {
    usb_v1::enqueueRx(received);
    if (!received.valid)
    {
      return;
    }

    const auto &packet = received.decoded;
    switch (packet.header)
    {
    case protocol::PacketHeader::CommandReceive:
    case protocol::PacketHeader::LiftoffDetection:
    case protocol::PacketHeader::EngineBurn:
    case protocol::PacketHeader::Control:
    case protocol::PacketHeader::Descent:
    case protocol::PacketHeader::RecoveryBeacon:
      updateStateLeds(packet.header);
      break;
    case protocol::PacketHeader::RecoveryLogData:
    case protocol::PacketHeader::ControlRollTelemetryV2:
    case protocol::PacketHeader::MissionLinkFallbackTelemetry:
      break;
    case protocol::PacketHeader::CommandResult:
    {
      const auto &result = packet.command_result;
#if GROUND_LORA_TIMING_DEBUG
      usb_v1::enqueuePrettyf(
          "GROUND_LORA_TIMING event=result_received transaction_id=%u command=0x%02X received_at_us=%lu",
          result.transaction_id, result.command,
          static_cast<unsigned long>(received.received_at_us));
#endif
      usb_v1::enqueuePrettyf(
          "CommandResult id=%u command=0x%02X phase=%s reason=%s detail=0x%08lX",
          result.transaction_id, result.command,
          protocol::phaseName(result.phase), protocol::reasonName(result.reason),
          static_cast<unsigned long>(result.detail));
      if (!markTransactionResult(result))
      {
        usb_v1::enqueuePrettyf(
            "unmatched CommandResult id=%u command=0x%02X",
            result.transaction_id, result.command);
      }
      break;
    }
    case protocol::PacketHeader::GroundTimeRequest:
      usb_v1::enqueuePrettyf(
          "GroundTimeRequest id=%u; reply with: time %u <unix> <ms>",
          packet.time_request.request_id, packet.time_request.request_id);
      break;
    }
  }

  void handleDecodeEvent(const DecodeEvent &event)
  {
    if (event.kind == DecodeEventKind::Packet)
    {
      handlePacket(event.packet);
    }
    else
    {
      usb_v1::enqueueFragment(event.fragment);
    }
  }
  UplinkWriteResult writeUplink(
      const protocol::UplinkFrame &uplink,
      const std::array<uint8_t, protocol::UPLINK_FRAME_SIZE> &application,
      const char *source, UplinkTiming &timing)
  {
    // 呼出側がframeをencodeし、送信直前のAUX Highを確認済みであること。
    const uint8_t prefix[] = {ADD_H, ADD_L, CHNNL};
    timing.write_started_at_us = micros();
    const std::size_t prefix_written = Serial1.write(prefix, sizeof(prefix));
    const std::size_t application_written =
        Serial1.write(application.data(), application.size());
    const bool write_complete = prefix_written == sizeof(prefix) &&
                                application_written == application.size();
    timing.write_finished_at_us = micros();
    if (digitalRead(aux) == LOW)
    {
      timing.aux_low_observed = true;
      timing.aux_low_at_us = micros();
    }
    Serial1.flush();
    timing.flush_finished_at_us = micros();
    if (!timing.aux_low_observed && digitalRead(aux) == LOW)
    {
      timing.aux_low_observed = true;
      timing.aux_low_at_us = micros();
    }
    const bool aux_completed = waitAuxHigh(AUX_TIMEOUT_MS);
    if (!write_complete)
    {
      return {false, usb_v1::TxError::UartWrite, millis()};
    }
    if (!aux_completed)
    {
      return {false, usb_v1::TxError::AuxTimeout, millis()};
    }
    timing.aux_high_at_us = micros();
    timing.completed_at_us = timing.aux_high_at_us;
#if GROUND_LORA_TIMING_DEBUG
    usb_v1::enqueuePrettyf("GROUND_LORA_TIMING source=%s transaction_id=%u requested_at_us=%lu dequeued_at_us=%lu boundary_kind=%s boundary_header=0x%02X boundary_sequence=%lu boundary_received_at_us=%lu boundary_age_us=%lu boundary_wait_us=%lu boundary_fallback=%u aux_ready_at_us=%lu write_started_at_us=%lu write_finished_at_us=%lu flush_finished_at_us=%lu aux_low_observed=%u aux_low_at_us=%lu aux_high_at_us=%lu completed_at_us=%lu",
                  source, uplink.transaction_id,
                  static_cast<unsigned long>(timing.requested_at_us),
                  static_cast<unsigned long>(timing.dequeued_at_us),
                  timing.boundary_observed
                      ? boundaryKindName(timing.boundary_kind)
                      : "none",
                  timing.boundary_header,
                  static_cast<unsigned long>(timing.boundary_sequence),
                  static_cast<unsigned long>(timing.boundary_received_at_us),
                  static_cast<unsigned long>(timing.boundary_age_us),
                  static_cast<unsigned long>(timing.boundary_wait_us),
                  timing.boundary_fallback ? 1U : 0U,
                  static_cast<unsigned long>(timing.aux_ready_at_us),
                  static_cast<unsigned long>(timing.write_started_at_us),
                  static_cast<unsigned long>(timing.write_finished_at_us),
                  static_cast<unsigned long>(timing.flush_finished_at_us),
                  timing.aux_low_observed ? 1U : 0U,
                  static_cast<unsigned long>(timing.aux_low_at_us),
                  static_cast<unsigned long>(timing.aux_high_at_us),
                  static_cast<unsigned long>(timing.completed_at_us));
#else
    (void)source;
#endif
    return {true, usb_v1::TxError::None, millis()};
  }

  bool takeEmergencyCommand(CommandMessage &message)
  {
    if (emergency_command_queue == nullptr ||
        xQueueReceive(emergency_command_queue, &message, 0) != pdPASS)
    {
      return false;
    }
    message.dequeued_at_us = micros();
    return true;
  }

  void sendCommandMessage(const CommandMessage &message)
  {
    UplinkTiming timing{};
    timing.requested_at_us = message.requested_at_us;
    timing.dequeued_at_us = message.dequeued_at_us;
    protocol::UplinkFrame uplink{};
    bool reserved = false;
    uint8_t transaction_id = 0;
    if (message.kind == CommandMessageKind::GroundTimeResponse)
    {
      transaction_id = message.command;
      uplink = {protocol::UplinkKind::GroundTimeResponse,
                transaction_id, 2, message.args};
    }
    else
    {
      protocol::UplinkKind kind = protocol::UplinkKind::MissionGeneric;
      uint8_t tracked_command = message.command;
      uint8_t wire_command = message.command;
      switch (message.kind)
      {
      case CommandMessageKind::MissionGeneric:
        kind = protocol::UplinkKind::MissionGeneric;
        break;
      case CommandMessageKind::ActuatorEmergency:
        kind = protocol::UplinkKind::ActuatorEmergency;
        tracked_command = COMMAND_RESULT_ACTUATOR_EMERGENCY;
        wire_command = 0;
        break;
      case CommandMessageKind::LiftoffEmergency:
        kind = protocol::UplinkKind::LiftoffDetectionEmergency;
        tracked_command = COMMAND_RESULT_LIFTOFF_EMERGENCY;
        wire_command = 0;
        break;
      case CommandMessageKind::ComBoardLocal:
        kind = protocol::UplinkKind::ComBoardLocal;
        break;
      default:
        return;
      }
      if (!reserveTransaction(kind, tracked_command, transaction_id))
      {
        usb_v1::enqueuePretty("no free transaction ID");
        return;
      }
      reserved = true;
      uplink = {kind, transaction_id, wire_command, message.args};
    }

    std::array<uint8_t, protocol::UPLINK_FRAME_SIZE> application{};
    if (!protocol::encodeUplink(uplink, application))
    {
      if (reserved)
      {
        releaseTransaction(transaction_id);
      }
      usb_v1::enqueuePrettyf("uplink failed id=%u", transaction_id);
      return;
    }

    const bool emergency =
        message.kind == CommandMessageKind::ActuatorEmergency ||
        message.kind == CommandMessageKind::LiftoffEmergency;
    bool tx_record_emitted = false;
    const auto emit_tx = [&](bool ok, usb_v1::TxError error,
                             uint32_t completed_at_ms) {
      if (tx_record_emitted)
      {
        return;
      }
      usb_v1::TxRecord tx_record{};
      tx_record.board_ms = completed_at_ms;
      tx_record.ok = ok;
      tx_record.kind = static_cast<uint8_t>(uplink.kind);
      tx_record.id = transaction_id;
      tx_record.command = uplink.command;
      tx_record.prefix = {ADD_H, ADD_L, CHNNL};
      tx_record.raw = application;
      tx_record.error = error;
      usb_v1::enqueueTx(tx_record);
      tx_record_emitted = true;
    };
    const auto fail_before_uplink = [&](const char *reason,
                                        const char *error_token) {
      if (reserved)
      {
        releaseTransaction(transaction_id);
      }
      usb_v1::SystemRecord system{};
      system.board_ms = millis();
      system.event = usb_v1::SystemEvent::UplinkAborted;
      system.kind = static_cast<uint8_t>(uplink.kind);
      system.id = transaction_id;
      system.command = uplink.command;
      std::snprintf(system.error.data(), system.error.size(), "%s", error_token);
      usb_v1::enqueueSystem(system);
      usb_v1::enqueuePrettyf("%s before uplink", reason);
      usb_v1::enqueuePrettyf("uplink failed id=%u", transaction_id);
    };

    UplinkBoundary boundary{};
    bool boundary_wait_started = false;
    uint32_t boundary_wait_started_at_ms = 0;
    uint32_t boundary_wait_started_at_us = 0;
    const auto record_aux_ready = [&]() {
      const uint32_t ready_at_ms = millis();
      const uint32_t ready_at_us = micros();
      timing.aux_ready_at_us = ready_at_us;
      if (!boundary_wait_started)
      {
        // invalidated境界やEmergency preemptを跨いでもdeadlineをresetしない。
        boundary_wait_started = true;
        boundary_wait_started_at_ms = ready_at_ms;
        boundary_wait_started_at_us = ready_at_us;
      }
    };
    while (true)
    {
      timing.boundary_observed = false;
      timing.boundary_fallback = false;
      timing.boundary_header = 0;
      timing.boundary_sequence = 0;
      timing.boundary_received_at_us = 0;
      timing.boundary_age_us = 0;
      CommandMessage pending_emergency{};

      // boundary受理後にblocking待機を残さないため、先にAUX readyを成立させる。
      if (emergency)
      {
        if (!waitAuxHigh(AUX_TIMEOUT_MS))
        {
          fail_before_uplink("AUX timeout", "AUX_TIMEOUT");
          return;
        }
        record_aux_ready();
      }
      else
      {
        const TxOpportunity aux_opportunity =
            waitForNormalTxOpportunity(pending_emergency);
        if (aux_opportunity == TxOpportunity::Timeout)
        {
#if GROUND_LORA_TIMING_DEBUG
          usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=pre_tx_timeout source=%s requested_at_us=%lu dequeued_at_us=%lu",
                        commandKindName(message.kind),
                        static_cast<unsigned long>(message.requested_at_us),
                        static_cast<unsigned long>(message.dequeued_at_us));
#endif
          fail_before_uplink("AUX timeout", "AUX_TIMEOUT");
          return;
        }
        if (aux_opportunity == TxOpportunity::EmergencyPending)
        {
#if GROUND_LORA_TIMING_DEBUG
          usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=normal_preempted stage=aux_wait normal=%s emergency=%s requested_at_us=%lu",
                        commandKindName(message.kind),
                        commandKindName(pending_emergency.kind),
                        static_cast<unsigned long>(pending_emergency.requested_at_us));
#endif
          sendCommandMessage(pending_emergency);
          continue;
        }
        record_aux_ready();
      }

      const TxOpportunity boundary_opportunity =
          waitForUplinkBoundary(
              message, pending_emergency, boundary, timing,
              boundary_wait_started_at_ms, boundary_wait_started_at_us);
      if (boundary_opportunity == TxOpportunity::EmergencyPending)
      {
#if GROUND_LORA_TIMING_DEBUG
        usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=normal_preempted stage=boundary_wait normal=%s emergency=%s requested_at_us=%lu",
                      commandKindName(message.kind),
                      commandKindName(pending_emergency.kind),
                      static_cast<unsigned long>(pending_emergency.requested_at_us));
#endif
        sendCommandMessage(pending_emergency);
        continue;
      }
      if (boundary_opportunity == TxOpportunity::Timeout)
      {
        if (!emergency)
        {
#if GROUND_LORA_TIMING_DEBUG
          usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=boundary_timeout source=%s timeout_ms=%lu requested_at_us=%lu dequeued_at_us=%lu boundary_wait_us=%lu",
                        commandKindName(message.kind),
                        static_cast<unsigned long>(UPLINK_BOUNDARY_TIMEOUT_MS),
                        static_cast<unsigned long>(message.requested_at_us),
                        static_cast<unsigned long>(message.dequeued_at_us),
                        static_cast<unsigned long>(timing.boundary_wait_us));
#endif
          fail_before_uplink("downlink boundary timeout",
                             "BOUNDARY_TIMEOUT");
          return;
        }
        // 2200 ms安全境界を得られない場合はavailabilityを優先して直接送る。
        // telemetry継続中でも、このfallbackはdownlinkと衝突し得る。
        timing.boundary_fallback = true;
#if GROUND_LORA_TIMING_DEBUG
        usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=emergency_boundary_fallback source=%s requested_at_us=%lu dequeued_at_us=%lu boundary_wait_us=%lu",
                      commandKindName(message.kind),
                      static_cast<unsigned long>(message.requested_at_us),
                      static_cast<unsigned long>(message.dequeued_at_us),
                      static_cast<unsigned long>(timing.boundary_wait_us));
#endif
        // timeout確定後はboundaryを再待機せず、AUX Highだけを有限待機する。
        if (!waitAuxHigh(AUX_TIMEOUT_MS))
        {
          fail_before_uplink("AUX timeout", "AUX_TIMEOUT");
          return;
        }
        record_aux_ready();
        break;
      }

      // boundary待機中にAUXがbusyへ戻った場合や、20 msの安全窓を外れた
      // 場合はこの境界を破棄する。transactionは保持して次の境界を待つ。
      if (boundary_opportunity == TxOpportunity::Ready)
      {
        const uint32_t commit_checked_at_us = micros();
        timing.boundary_age_us = uplink_boundary_policy::elapsedUs(
            commit_checked_at_us, boundary.received_at_us);
        const bool aux_still_high = digitalRead(aux) == HIGH;
        const bool boundary_still_fresh = uplink_boundary_policy::isFresh(
            commit_checked_at_us, boundary.received_at_us,
            UPLINK_BOUNDARY_FRESH_US);
        if (!aux_still_high || !boundary_still_fresh)
        {
#if GROUND_LORA_TIMING_DEBUG
          usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=boundary_invalidated source=%s aux_high=%u boundary_age_us=%lu boundary_sequence=%lu",
                        commandKindName(message.kind),
                        aux_still_high ? 1U : 0U,
                        static_cast<unsigned long>(timing.boundary_age_us),
                        static_cast<unsigned long>(timing.boundary_sequence));
#endif
          continue;
        }
      }

      // ここを通常UART TXのcommit pointとする。Queue受信とUART writeを
      // 同期atomicにはせず、poll後のEmergencyは進行中TXをpreemptしない。
      if (!emergency && takeEmergencyCommand(pending_emergency))
      {
#if GROUND_LORA_TIMING_DEBUG
        usb_v1::enqueuePrettyf("GROUND_LORA_TIMING event=normal_preempted stage=uart_commit normal=%s emergency=%s requested_at_us=%lu",
                      commandKindName(message.kind),
                      commandKindName(pending_emergency.kind),
                      static_cast<unsigned long>(pending_emergency.requested_at_us));
#endif
        // 選択済み通常transactionは保持し、次の単一境界から再開する。
        sendCommandMessage(pending_emergency);
        continue;
      }
      break;
    }

    // commit point以降の一回のUART送信はnonpreemptibleとする。
    const UplinkWriteResult write_result =
        writeUplink(uplink, application, commandKindName(message.kind), timing);
    emit_tx(write_result.ok, write_result.error, write_result.completed_at_ms);
    if (!write_result.ok)
    {
      if (reserved)
      {
        releaseTransaction(transaction_id);
      }
      usb_v1::enqueuePrettyf("uplink failed id=%u", transaction_id);
      return;
    }
    usb_v1::enqueuePrettyf("uplink sent kind=%u id=%u command=0x%02X",
                           static_cast<uint8_t>(uplink.kind), transaction_id,
                           uplink.command);
  }

  void commandSendTask(void *)
  {
    for (;;)
    {
      CommandMessage message{};
      if (takeEmergencyCommand(message))
      {
        sendCommandMessage(message);
        continue;
      }
      if (xQueueReceive(command_queue, &message, pdMS_TO_TICKS(10)) != pdPASS)
      {
        continue;
      }
      message.dequeued_at_us = micros();
      if (message.kind == CommandMessageKind::ReleaseTransaction)
      {
        const bool released = releaseTransaction(message.command);
        usb_v1::SystemRecord system{};
        system.board_ms = millis();
        system.event = usb_v1::SystemEvent::TransactionRelease;
        system.id = message.command;
        system.ok = released;
        usb_v1::enqueueSystem(system);
        usb_v1::enqueuePrettyf("transaction id=%u %s", message.command,
                               released ? "released" : "was not pending");
        continue;
      }

      sendCommandMessage(message);
    }
  }

  void printPacketTask(void *)
  {
    uint32_t reported_drops = 0;
    for (;;)
    {
      DecodeEvent event{};
      if (receive_decode_event(event, portMAX_DELAY))
      {
        handleDecodeEvent(event);
      }
      const uint32_t dropped = dropped_packet_count();
      if (dropped != reported_drops)
      {
        usb_v1::SystemRecord overflow{};
        overflow.board_ms = millis();
        overflow.event = usb_v1::SystemEvent::QueueOverflow;
        overflow.count = dropped - reported_drops;
        std::snprintf(overflow.source.data(), overflow.source.size(),
                      "DECODE_EVENT");
        usb_v1::enqueueSystem(overflow);
        reported_drops = dropped;
      }
    }
  }

  void setupLoraSettings()
  {
    digitalWrite(m0, HIGH);
    digitalWrite(m1, HIGH);
    delay(100);
    while (Serial1.available() > 0)
    {
      Serial1.read();
    }
    Serial.println("# Writing LoRa settings...");
    Serial1.write(settingCmd, sizeof(settingCmd));
    Serial1.flush();
    delay(200);
    Serial.print("# LoRa setup response:");
    while (Serial1.available() > 0)
    {
      const uint8_t value = static_cast<uint8_t>(Serial1.read());
      if (value < 0x10)
      {
        Serial.print('0');
      }
      Serial.print(value, HEX);
      Serial.print(' ');
    }
    Serial.println();
    Serial.println("# LoRa setup finished. Restore Communication mode and upload again.");
  }

  void readLoraSettings()
  {
    digitalWrite(m0, HIGH);
    digitalWrite(m1, HIGH);
    delay(100);

    bool received = false;
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("# LoRa readback: AUX timeout before command");
    }
    else
    {
      while (Serial1.available() > 0)
      {
        Serial1.read();
      }

      // 設定を書き換えず、レジスタ読出しコマンドだけを送る。
      Serial1.write(readCmd, sizeof(readCmd));
      Serial1.flush();
      if (!waitAuxHigh(AUX_TIMEOUT_MS))
      {
        Serial.println("# LoRa readback: AUX timeout after command");
      }

      Serial.print("# LoRa readback raw:");
      const uint32_t started_at = millis();
      uint32_t last_received_at = started_at;
      while (millis() - started_at < 2000)
      {
        while (Serial1.available() > 0)
        {
          const uint8_t value = static_cast<uint8_t>(Serial1.read());
          Serial.printf(" %02X", value);
          received = true;
          last_received_at = millis();
        }
        if (received && millis() - last_received_at >= 20)
        {
          break;
        }
        delay(1);
      }
      Serial.println();
      if (!received)
      {
        Serial.println("# LoRa readback: no response");
      }
    }

    // 診断の成否にかかわらず通常通信状態へ戻す。
    digitalWrite(m0, LOW);
    digitalWrite(m1, LOW);
    delay(100);
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("# LoRa readback: AUX timeout while restoring communication mode");
    }
    Serial1.end();
    Serial1.begin(115200, SERIAL_8N1, LoRA_RX, LoRA_TX);
    Serial.println("# LoRa readback finished; communication mode restored");
  }
} // 無名名前空間

void setup()
{
  Serial.begin(115200);
  pinMode(aux, INPUT);
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(top_led, OUTPUT);
  pinMode(liftoff_led, OUTPUT);
  pinMode(control_led, OUTPUT);
  pinMode(update_led, OUTPUT);

  if (BOOT_MODE == BootMode::LoRaSetup || BOOT_MODE == BootMode::LoRaReadback)
  {
    Serial1.begin(9600, SERIAL_8N1, LoRA_RX, LoRA_TX);
    if (BOOT_MODE == BootMode::LoRaSetup)
    {
      setupLoraSettings();
    }
    else
    {
      readLoraSettings();
    }
    return;
  }

  Serial1.begin(115200, SERIAL_8N1, LoRA_RX, LoRA_TX);
  digitalWrite(m0, LOW);
  digitalWrite(m1, LOW);
  digitalWrite(top_led, LOW);
  digitalWrite(liftoff_led, LOW);
  digitalWrite(control_led, LOW);
  digitalWrite(update_led, LOW);

  if (!usb_v1::initialize())
  {
    return;
  }
  usb_v1::SystemRecord boot{};
  boot.board_ms = millis();
  boot.event = usb_v1::SystemEvent::Boot;
  usb_v1::enqueueSystem(boot);

  const auto report_init_failure = [](const char *task) {
    usb_v1::SystemRecord failure{};
    failure.board_ms = millis();
    failure.event = usb_v1::SystemEvent::TaskInitFailed;
    std::snprintf(failure.task.data(), failure.task.size(), "%s", task);
    std::snprintf(failure.error.data(), failure.error.size(), "NO_MEMORY");
    usb_v1::enqueueSystem(failure);
  };

  command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  if (command_queue == nullptr)
  {
    report_init_failure("COMMAND_QUEUE");
    return;
  }
  emergency_command_queue =
      xQueueCreate(EMERGENCY_COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  if (emergency_command_queue == nullptr)
  {
    report_init_failure("EMERGENCY_QUEUE");
    return;
  }
  transaction_mutex = xSemaphoreCreateMutex();
  if (transaction_mutex == nullptr)
  {
    report_init_failure("TRANSACTION_MUTEX");
    return;
  }
  if (!start_decode_task())
  {
    report_init_failure("DECODE");
    return;
  }
  const bool print_task_ready = xTaskCreateUniversal(
                                    printPacketTask, "print_packet_task", 6144,
                                    nullptr, 1, nullptr, 0) == pdPASS;
  const bool command_task_ready = print_task_ready &&
                                  xTaskCreateUniversal(
                                      commandSendTask, "command_send_task", 4096,
                                      nullptr, 2, nullptr, 0) == pdPASS;
  if (!print_task_ready || !command_task_ready)
  {
    usb_v1::SystemRecord failure{};
    failure.board_ms = millis();
    failure.event = usb_v1::SystemEvent::TaskInitFailed;
    std::snprintf(failure.task.data(), failure.task.size(), "%s",
                  print_task_ready ? "COMMAND" : "PACKET");
    std::snprintf(failure.error.data(), failure.error.size(), "NO_MEMORY");
    usb_v1::enqueueSystem(failure);
    return;
  }
  usb_v1::SystemRecord ready{};
  ready.board_ms = millis();
  ready.event = usb_v1::SystemEvent::Ready;
  usb_v1::enqueueSystem(ready);
  printUsage();
}

void loop()
{
  if (BOOT_MODE != BootMode::Communication)
  {
    delay(1000);
    return;
  }
  readConsole();
  delay(10);
}
