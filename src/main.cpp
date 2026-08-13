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

namespace
{
  constexpr UBaseType_t COMMAND_QUEUE_LENGTH = 16;
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
  };

  constexpr bool requiresUplinkWindow(CommandMessageKind kind)
  {
    return kind != CommandMessageKind::ActuatorEmergency &&
           kind != CommandMessageKind::LiftoffEmergency;
  }

  static_assert(!requiresUplinkWindow(CommandMessageKind::ActuatorEmergency));
  static_assert(!requiresUplinkWindow(CommandMessageKind::LiftoffEmergency));
  static_assert(requiresUplinkWindow(CommandMessageKind::MissionGeneric));

  QueueHandle_t command_queue = nullptr;
  SemaphoreHandle_t uplink_window_signal = nullptr;
  SemaphoreHandle_t transaction_mutex = nullptr;
  TaskHandle_t command_send_task_handle = nullptr;
  protocol::TransactionTracker transaction_tracker;
  char console_line[CONSOLE_LINE_SIZE] = {};
  std::size_t console_line_length = 0;
  constexpr uint32_t ACTUATOR_EMERGENCY_NOTIFICATION = 1U << 0U;
  constexpr uint32_t LIFTOFF_EMERGENCY_NOTIFICATION = 1U << 1U;

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

  bool notifyEmergency(CommandMessageKind kind)
  {
    if (command_send_task_handle == nullptr)
    {
      return false;
    }
    const uint32_t notification =
        kind == CommandMessageKind::ActuatorEmergency
            ? ACTUATOR_EMERGENCY_NOTIFICATION
            : LIFTOFF_EMERGENCY_NOTIFICATION;
    return xTaskNotify(command_send_task_handle, notification, eSetBits) == pdPASS;
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
      if (!notifyEmergency(message.kind))
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

  bool writeUplink(const protocol::UplinkFrame &uplink)
  {
    std::array<uint8_t, protocol::UPLINK_FRAME_SIZE> application{};
    if (!protocol::encodeUplink(uplink, application))
    {
      return false;
    }
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("AUX timeout before uplink");
      return false;
    }
    const uint8_t prefix[] = {ADD_H, ADD_L, CHNNL};
    Serial1.write(prefix, sizeof(prefix));
    Serial1.write(application.data(), application.size());
    Serial1.flush();
    if (!waitAuxHigh(AUX_TIMEOUT_MS))
    {
      Serial.println("AUX timeout after uplink");
      return false;
    }
    return true;
  }

  bool takeEmergencyNotification(CommandMessage &message)
  {
    uint32_t notification = 0;
    if (xTaskNotifyWait(0, UINT32_MAX, &notification, 0) != pdTRUE)
    {
      return false;
    }
    if ((notification & ACTUATOR_EMERGENCY_NOTIFICATION) != 0)
    {
      message.kind = CommandMessageKind::ActuatorEmergency;
      if ((notification & LIFTOFF_EMERGENCY_NOTIFICATION) != 0)
      {
        xTaskNotify(command_send_task_handle, LIFTOFF_EMERGENCY_NOTIFICATION,
                    eSetBits);
      }
      return true;
    }
    message.kind = CommandMessageKind::LiftoffEmergency;
    return true;
  }

  void sendCommandMessage(const CommandMessage &message)
  {
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

    // downlink直後のComBoard受信窓へ送信を合わせ、half-duplex衝突を避ける。
    // Emergencyもfreshな受信窓を有限時間待つが、窓が来なくても必ず送信を試みる。
    if (requiresUplinkWindow(message.kind))
    {
      (void)xSemaphoreTake(uplink_window_signal, 0);
      const uint32_t started_at = millis();
      bool window_opened = false;
      while (millis() - started_at < UPLINK_WINDOW_TIMEOUT_MS)
      {
        CommandMessage emergency{};
        if (takeEmergencyNotification(emergency))
        {
          sendCommandMessage(emergency);
          continue;
        }
        if (xSemaphoreTake(uplink_window_signal, pdMS_TO_TICKS(10)) == pdTRUE)
        {
          window_opened = true;
          break;
        }
      }
      if (!window_opened)
      {
        if (reserved)
        {
          releaseTransaction(transaction_id);
        }
        Serial.printf("uplink window timeout id=%u\r\n", transaction_id);
        return;
      }
    }
    else
    {
      (void)xSemaphoreTake(uplink_window_signal, 0);
      (void)xSemaphoreTake(uplink_window_signal,
                           pdMS_TO_TICKS(UPLINK_WINDOW_TIMEOUT_MS));
    }
    if (!writeUplink(uplink))
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
      if (takeEmergencyNotification(message))
      {
        sendCommandMessage(message);
        continue;
      }
      if (xQueueReceive(command_queue, &message, pdMS_TO_TICKS(10)) != pdPASS)
      {
        continue;
      }
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
        xSemaphoreGive(uplink_window_signal);
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
  uplink_window_signal = xSemaphoreCreateBinary();
  transaction_mutex = xSemaphoreCreateMutex();
  if (command_queue == nullptr || uplink_window_signal == nullptr ||
      transaction_mutex == nullptr || !start_decode_task())
  {
    Serial.println("task queue initialization failed");
    return;
  }
  if (xTaskCreateUniversal(
          printPacketTask, "print_packet_task", 6144, nullptr, 1, nullptr, 0) != pdPASS ||
      xTaskCreateUniversal(
          commandSendTask, "command_send_task", 4096, nullptr, 2,
          &command_send_task_handle, 0) != pdPASS)
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
