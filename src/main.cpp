#include <Arduino.h>
#include <array>
#include <cctype>
#include <cerrno>
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
      Serial.println("command queue full");
      return false;
    }
    if (xQueueSend(command_queue, &message, 0) != pdPASS)
    {
      Serial.println("command queue full");
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
    Serial.println("commands:");
    Serial.println("  g <command> [arg0 ... arg5]");
    Serial.println("  ae | le");
    Serial.println("  local <command> [arg0 ... arg5]");
    Serial.println("  time <request_id> <unix_seconds> <milliseconds>");
    Serial.println("  release <transaction_id>");
    Serial.println("values accept decimal or 0x-prefixed hexadecimal");
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
        Serial.println("unexpected emergency argument");
        return;
      }
      message.kind = std::strcmp(operation, "ae") == 0
                         ? CommandMessageKind::ActuatorEmergency
                         : CommandMessageKind::LiftoffEmergency;
      if (!notifyEmergency(message.kind, message.requested_at_us))
      {
        Serial.println("emergency notification failed");
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
        Serial.println("usage: time <request_id 1..255> <unix_seconds> <milliseconds 0..999>");
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
        Serial.println("usage: release <transaction_id 1..255>");
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
      Serial.println("unknown command; type help");
      return;
    }
    char *command_text = strtok_r(nullptr, " \t", &save);
    uint32_t command = 0;
    bool valid = parseUnsigned(command_text, UINT8_MAX, command);
    parseArgs(save, message.args, valid);
    if (!valid)
    {
      Serial.println("invalid command or argument");
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
        Serial.println("console line too long");
        continue;
      }
      console_line[console_line_length++] = value;
    }
  }

  void printSemantic(
      const char *label,
      protocol::SemanticValue value,
      double scale,
      double offset,
      const char *unit)
  {
    Serial.print(label);
    Serial.print(": ");
    if (!value.numeric)
    {
      Serial.println(value.status);
      return;
    }
    Serial.print(offset + static_cast<double>(value.count) * scale, 3);
    if (unit[0] != '\0')
    {
      Serial.print(' ');
      Serial.print(unit);
    }
    Serial.println();
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

  void printPosition(uint16_t east, uint16_t north, uint16_t height)
  {
    printSemantic("GNSS East", protocol::decodeGnssCoordinate(east), 1.0, 0.0, "m");
    printSemantic("GNSS North", protocol::decodeGnssCoordinate(north), 1.0, 0.0, "m");
    printSemantic("GNSS height", protocol::decodeGnssHeight(height), 5.0, -100.0, "m");
  }

  void printFlight(const protocol::DecodedPacket &packet)
  {
    const auto &value = packet.flight;
    Serial.printf("Flight status: 0x%04X\r\n", value.status);
    printSemantic("Roll", protocol::decodeRoll(value.roll), 0.5, 0.0, "deg");
    printSemantic("Roll rate", protocol::decodeRollRate(value.roll_rate), 0.1, 0.0, "deg/s");
    printSemantic("Tilt", protocol::decodeTiltMagnitude(value.tilt_magnitude), 0.75, 0.0, "deg");
    printSemantic("Tilt direction", protocol::decodeTiltDirection(value.tilt_direction), 1.0, 0.0, "deg");
    printSemantic("Fin angle", protocol::decodeFinAngle(value.fin_angle), 0.125, -15.0, "deg");
    printSemantic("Fin rate", protocol::decodeFinRate(value.fin_rate), 0.02, 0.0, "deg/s");
    printSemantic("LPS pressure", protocol::decodePressure(value.pressure), 0.2, 800.0, "hPa");
    printSemantic("LPS temperature", protocol::decodeTemperature(value.temperature), 1.0, -50.0, "degC");
    printSemantic("Airspeed", protocol::decodeAirspeed(value.airspeed), 1.0, 0.0, "m/s");
    // TODO(SIMULATION): requested torque scale 0.002 N m/LSBを確定する。
    printSemantic("Requested torque", protocol::decodeRequestedTorque(value.requested_torque), 0.002, 0.0, "N m");
    printSemantic("Flight elapsed", protocol::decodeFlightElapsed(value.elapsed), 0.1, 0.0, "s");
    printPosition(value.east, value.north, value.height);
  }

  void printCommandReceive(const protocol::CommandReceiveTelemetry &value)
  {
    static const char *const status_names[24] = {
        "ICM", "LPS", "SSC", "AS5047D", "STS", "Fin zero",
        "Para open", "Para close", "Logic battery", "Motor battery",
        "Mission SD", "Com SD", "CAN", "Persistence", "Fin busy",
        "Para busy", "Gyro bias", "Gravity reference", "SSC zero",
        "Flash data", "Flash health", "Profile valid", "Fin disabled",
        "Calibration busy"};
    Serial.printf("CommandReceive status: 0x%06lX profile=%u fin_mode=%s para_mode=%s\r\n",
                  static_cast<unsigned long>(value.status),
                  value.motor_profile,
                  protocol::finModeName(value.fin_mode),
                  protocol::paraModeName(value.para_mode));
    for (uint8_t bit = 0; bit < 24; ++bit)
    {
      Serial.printf("  %-18s: %s\r\n", status_names[bit],
                    (value.status & (1UL << bit)) != 0 ? "YES" : "NO");
    }
    printSemantic("Tilt", protocol::decodeTiltMagnitude(value.tilt_magnitude), 0.75, 0.0, "deg");
    printSemantic("Tilt direction", protocol::decodeTiltDirection(value.tilt_direction), 1.0, 0.0, "deg");
    printSemantic("Fin angle", protocol::decodeFinAngle(value.fin_angle), 0.125, -15.0, "deg");
    printSemantic("Parachute angle", protocol::decodeParaAngle(value.para_angle), 1.5, 0.0, "deg");
    printSemantic("LPS pressure", protocol::decodePressure(value.pressure), 0.2, 800.0, "hPa");
    printSemantic("LPS temperature", protocol::decodeTemperature(value.temperature), 1.0, -50.0, "degC");
    printSemantic("Airspeed", protocol::decodeAirspeed(value.airspeed), 1.0, 0.0, "m/s");
    printSemantic("Logic voltage", protocol::decodeBattery(value.logic_voltage), 0.05, 0.0, "V");
    printSemantic("Motor voltage", protocol::decodeBattery(value.motor_voltage), 0.05, 0.0, "V");
    printPosition(value.east, value.north, value.height);
  }

  void printDescent(const protocol::DescentTelemetry &value)
  {
    Serial.printf("Descent status: 0x%04X\r\n", value.status);
    static const char *const status_names[] = {
        "LPS deployment", "Elapsed deployment", "Power cutoff",
        "Com SD", "Mission-CAN", "Deployment shock", "STS overload",
        "STS overcurrent", "STS overtemperature", "STS encoder fault",
        "STS voltage fault"};
    Serial.printf("  Parachute state: %u\r\n", (value.status >> 2U) & 0x03U);
    for (uint8_t index = 0; index < 2; ++index)
      Serial.printf("  %-19s: %s\r\n", status_names[index],
                    (value.status & (1U << index)) != 0 ? "YES" : "NO");
    for (uint8_t bit = 4; bit <= 12; ++bit)
      Serial.printf("  %-19s: %s\r\n", status_names[bit - 2],
                    (value.status & (1U << bit)) != 0 ? "YES" : "NO");
    printSemantic("LPS pressure", protocol::decodePressure(value.pressure), 0.2, 800.0, "hPa");
    printSemantic("LPS temperature", protocol::decodeTemperature(value.temperature), 1.0, -50.0, "degC");
    printSemantic("Parachute angle", protocol::decodeParaAngle(value.para_angle), 1.5, 0.0, "deg");
    printSemantic("Descent elapsed", protocol::decodeLongElapsed(value.elapsed), 0.1, 0.0, "s");
    printPosition(value.east, value.north, value.height);
  }

  void printRecovery(const protocol::RecoveryBeacon &value)
  {
    printSemantic("Logic voltage", protocol::decodeBattery(value.logic_voltage), 0.05, 0.0, "V");
    printSemantic("Motor voltage", protocol::decodeBattery(value.motor_voltage), 0.05, 0.0, "V");
    printPosition(value.east, value.north, value.height);
    printSemantic("Recovery elapsed", protocol::decodeLongElapsed(value.elapsed), 10.0, 0.0, "s");
  }

  void printPacket(const ReceivedPacket &received)
  {
    const auto &packet = received.packet;
    Serial.printf("\r\npacket header=0x%02X\r\n", static_cast<uint8_t>(packet.header));
    if (received.has_receive_interval)
    {
      Serial.printf("Receive interval: %.3f s\r\n", received.receive_interval_ms / 1000.0);
    }
    else
    {
      Serial.println("Receive interval: N/A");
    }
    if (received.has_rssi)
    {
      Serial.printf("RSSI: %d dBm\r\n", static_cast<int>(received.rssi) - 256);
    }
    else
    {
      Serial.println("RSSI: unavailable");
    }

    switch (packet.header)
    {
    case protocol::PacketHeader::CommandReceive:
      updateStateLeds(packet.header);
      printCommandReceive(packet.command_receive);
      break;
    case protocol::PacketHeader::LiftoffDetection:
    case protocol::PacketHeader::EngineBurn:
    case protocol::PacketHeader::Control:
      updateStateLeds(packet.header);
      printFlight(packet);
      break;
    case protocol::PacketHeader::Descent:
      updateStateLeds(packet.header);
      printDescent(packet.descent);
      break;
    case protocol::PacketHeader::RecoveryBeacon:
      updateStateLeds(packet.header);
      printRecovery(packet.recovery);
      break;
    case protocol::PacketHeader::RecoveryLogData:
      Serial.printf("Recovery log transfer=%u source=%u eof=%u offset=%lu length=%u data=",
                    packet.recovery_log.transfer_id,
                    packet.recovery_log.meta & 1U,
                    (packet.recovery_log.meta >> 1U) & 1U,
                    static_cast<unsigned long>(packet.recovery_log.offset),
                    packet.recovery_log.data_length);
      for (uint8_t index = 0; index < packet.recovery_log.data_length; ++index)
      {
        Serial.printf("%02X", packet.recovery_log.data[index]);
      }
      Serial.println();
      break;
    case protocol::PacketHeader::CommandResult:
    {
      const auto &result = packet.command_result;
#if GROUND_LORA_TIMING_DEBUG
      Serial.printf("GROUND_LORA_TIMING event=result_received transaction_id=%u command=0x%02X received_at_us=%lu\r\n",
                    result.transaction_id, result.command,
                    static_cast<unsigned long>(received.received_at_us));
#endif
      Serial.printf("CommandResult id=%u command=0x%02X phase=%s reason=%s detail=0x%08lX\r\n",
                    result.transaction_id, result.command,
                    protocol::phaseName(result.phase),
                    protocol::reasonName(result.reason),
                    static_cast<unsigned long>(result.detail));
      if (!markTransactionResult(result))
      {
        Serial.printf("unmatched CommandResult id=%u command=0x%02X\r\n",
                      result.transaction_id, result.command);
      }
      break;
    }
    case protocol::PacketHeader::GroundTimeRequest:
      Serial.printf("GroundTimeRequest id=%u; reply with: time %u <unix> <ms>\r\n",
                    packet.time_request.request_id,
                    packet.time_request.request_id);
      break;
    }
    const uint32_t dropped = dropped_packet_count();
    if (dropped != 0)
    {
      Serial.printf("Ground packet queue drops: %lu\r\n",
                    static_cast<unsigned long>(dropped));
    }
  }

  bool writeUplink(const protocol::UplinkFrame &uplink,
                   const std::array<uint8_t, protocol::UPLINK_FRAME_SIZE> &application,
                   const char *source,
                   UplinkTiming &timing)
  {
    // 呼出側がframeをencodeし、送信直前のAUX Highを確認済みであること。
    const uint8_t prefix[] = {ADD_H, ADD_L, CHNNL};
    timing.write_started_at_us = micros();
    Serial1.write(prefix, sizeof(prefix));
    Serial1.write(application.data(), application.size());
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
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("AUX timeout after uplink");
      return false;
    }
    timing.aux_high_at_us = micros();
    timing.completed_at_us = timing.aux_high_at_us;
#if GROUND_LORA_TIMING_DEBUG
    Serial.printf("GROUND_LORA_TIMING source=%s transaction_id=%u requested_at_us=%lu dequeued_at_us=%lu boundary_kind=%s boundary_header=0x%02X boundary_sequence=%lu boundary_received_at_us=%lu boundary_age_us=%lu boundary_wait_us=%lu boundary_fallback=%u aux_ready_at_us=%lu write_started_at_us=%lu write_finished_at_us=%lu flush_finished_at_us=%lu aux_low_observed=%u aux_low_at_us=%lu aux_high_at_us=%lu completed_at_us=%lu\r\n",
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
    return true;
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
        Serial.println("no free transaction ID");
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
      Serial.printf("uplink failed id=%u\r\n", transaction_id);
      return;
    }

    const bool emergency =
        message.kind == CommandMessageKind::ActuatorEmergency ||
        message.kind == CommandMessageKind::LiftoffEmergency;
    const auto fail_before_uplink = [&](const char *reason) {
      if (reserved)
      {
        releaseTransaction(transaction_id);
      }
      Serial.printf("%s before uplink\r\n", reason);
      Serial.printf("uplink failed id=%u\r\n", transaction_id);
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
          fail_before_uplink("AUX timeout");
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
          Serial.printf("GROUND_LORA_TIMING event=pre_tx_timeout source=%s requested_at_us=%lu dequeued_at_us=%lu\r\n",
                        commandKindName(message.kind),
                        static_cast<unsigned long>(message.requested_at_us),
                        static_cast<unsigned long>(message.dequeued_at_us));
#endif
          fail_before_uplink("AUX timeout");
          return;
        }
        if (aux_opportunity == TxOpportunity::EmergencyPending)
        {
#if GROUND_LORA_TIMING_DEBUG
          Serial.printf("GROUND_LORA_TIMING event=normal_preempted stage=aux_wait normal=%s emergency=%s requested_at_us=%lu\r\n",
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
        Serial.printf("GROUND_LORA_TIMING event=normal_preempted stage=boundary_wait normal=%s emergency=%s requested_at_us=%lu\r\n",
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
          Serial.printf("GROUND_LORA_TIMING event=boundary_timeout source=%s timeout_ms=%lu requested_at_us=%lu dequeued_at_us=%lu boundary_wait_us=%lu\r\n",
                        commandKindName(message.kind),
                        static_cast<unsigned long>(UPLINK_BOUNDARY_TIMEOUT_MS),
                        static_cast<unsigned long>(message.requested_at_us),
                        static_cast<unsigned long>(message.dequeued_at_us),
                        static_cast<unsigned long>(timing.boundary_wait_us));
#endif
          fail_before_uplink("downlink boundary timeout");
          return;
        }
        // 2200 ms安全境界を得られない場合はavailabilityを優先して直接送る。
        // telemetry継続中でも、このfallbackはdownlinkと衝突し得る。
        timing.boundary_fallback = true;
#if GROUND_LORA_TIMING_DEBUG
        Serial.printf("GROUND_LORA_TIMING event=emergency_boundary_fallback source=%s requested_at_us=%lu dequeued_at_us=%lu boundary_wait_us=%lu\r\n",
                      commandKindName(message.kind),
                      static_cast<unsigned long>(message.requested_at_us),
                      static_cast<unsigned long>(message.dequeued_at_us),
                      static_cast<unsigned long>(timing.boundary_wait_us));
#endif
        // timeout確定後はboundaryを再待機せず、AUX Highだけを有限待機する。
        if (!waitAuxHigh(AUX_TIMEOUT_MS))
        {
          fail_before_uplink("AUX timeout");
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
          Serial.printf("GROUND_LORA_TIMING event=boundary_invalidated source=%s aux_high=%u boundary_age_us=%lu boundary_sequence=%lu\r\n",
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
        Serial.printf("GROUND_LORA_TIMING event=normal_preempted stage=uart_commit normal=%s emergency=%s requested_at_us=%lu\r\n",
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
    if (!writeUplink(uplink, application, commandKindName(message.kind), timing))
    {
      if (reserved)
      {
        releaseTransaction(transaction_id);
      }
      Serial.printf("uplink failed id=%u\r\n", transaction_id);
      return;
    }
    Serial.printf("uplink sent kind=%u id=%u command=0x%02X\r\n",
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
        Serial.printf("transaction id=%u %s\r\n", message.command,
                      releaseTransaction(message.command)
                          ? "released"
                          : "was not pending");
        continue;
      }

      sendCommandMessage(message);
    }
  }

  void printPacketTask(void *)
  {
    for (;;)
    {
      ReceivedPacket packet{};
      if (receive_packet(packet, portMAX_DELAY))
      {
        printPacket(packet);
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
    Serial.println("Writing LoRa settings...");
    Serial1.write(settingCmd, sizeof(settingCmd));
    Serial1.flush();
    delay(200);
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
    Serial.println("LoRa setup finished. Restore Communication mode and upload again.");
  }

  void readLoraSettings()
  {
    digitalWrite(m0, HIGH);
    digitalWrite(m1, HIGH);
    delay(100);

    bool received = false;
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("LoRa readback: AUX timeout before command");
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
        Serial.println("LoRa readback: AUX timeout after command");
      }

      Serial.print("LoRa readback raw:");
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
        Serial.println("LoRa readback: no response");
      }
    }

    // 診断の成否にかかわらず通常通信状態へ戻す。
    digitalWrite(m0, LOW);
    digitalWrite(m1, LOW);
    delay(100);
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("LoRa readback: AUX timeout while restoring communication mode");
    }
    Serial1.end();
    Serial1.begin(115200, SERIAL_8N1, LoRA_RX, LoRA_TX);
    Serial.println("LoRa readback finished; communication mode restored");
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

  command_queue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  emergency_command_queue =
      xQueueCreate(EMERGENCY_COMMAND_QUEUE_LENGTH, sizeof(CommandMessage));
  transaction_mutex = xSemaphoreCreateMutex();
  if (command_queue == nullptr || emergency_command_queue == nullptr ||
      transaction_mutex == nullptr || !start_decode_task())
  {
    Serial.println("task queue initialization failed");
    return;
  }
  if (xTaskCreateUniversal(
          printPacketTask, "print_packet_task", 6144, nullptr, 1, nullptr, 0) != pdPASS ||
      xTaskCreateUniversal(
          commandSendTask, "command_send_task", 4096, nullptr, 2, nullptr, 0) != pdPASS)
  {
    Serial.println("task creation failed");
    return;
  }
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
