#include <Arduino.h>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
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
    Result,
  };

  struct CommandMessage
  {
    CommandMessageKind kind;
    uint8_t command;
    std::array<uint8_t, 6> args;
    protocol::CommandResult result;
  };

  QueueHandle_t command_queue = nullptr;
  protocol::TransactionTracker transaction_tracker;
  char console_line[CONSOLE_LINE_SIZE] = {};
  std::size_t console_line_length = 0;

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
    if (command_queue == nullptr ||
        xQueueSend(command_queue, &message, 0) != pdPASS)
    {
      Serial.println("command queue full");
      return false;
    }
    return true;
  }

  void printUsage()
  {
    Serial.println("commands:");
    Serial.println("  g <command> [arg0 ... arg5]");
    Serial.println("  ae | le");
    Serial.println("  local <command> [arg0 ... arg5]");
    Serial.println("  time <request_id> <unix_seconds> <milliseconds>");
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
      enqueueCommand(message);
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
    Serial.printf("CommandReceive status: 0x%06lX profile=%u fin_mode=%u para_mode=%u\r\n",
                  static_cast<unsigned long>(value.status),
                  value.motor_profile, value.fin_mode, value.para_mode);
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
      CommandMessage message{};
      message.kind = CommandMessageKind::Result;
      message.result = result;
      enqueueCommand(message);
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

  void commandSendTask(void *)
  {
    for (;;)
    {
      CommandMessage message{};
      if (xQueueReceive(command_queue, &message, portMAX_DELAY) != pdPASS)
      {
        continue;
      }
      if (message.kind == CommandMessageKind::Result)
      {
        if (!transaction_tracker.markResult(message.result))
        {
          Serial.printf("unmatched CommandResult id=%u command=0x%02X\r\n",
                        message.result.transaction_id, message.result.command);
        }
        continue;
      }

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
          break;
        }
        if (!transaction_tracker.reserve(kind, tracked_command, transaction_id))
        {
          Serial.println("no free transaction ID");
          continue;
        }
        reserved = true;
        uplink = {kind, transaction_id, wire_command, message.args};
      }

      if (!writeUplink(uplink))
      {
        if (reserved)
        {
          transaction_tracker.release(transaction_id);
        }
        Serial.printf("uplink failed id=%u\r\n", transaction_id);
        continue;
      }
      Serial.printf("uplink sent kind=%u id=%u command=0x%02X\r\n",
                    static_cast<uint8_t>(uplink.kind), transaction_id,
                    uplink.command);
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

  if (BOOT_MODE == BootMode::LoRaSetup)
  {
    Serial1.begin(9600, SERIAL_8N1, LoRA_RX, LoRA_TX);
    setupLoraSettings();
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
  if (command_queue == nullptr || !start_decode_task())
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
  if (BOOT_MODE == BootMode::LoRaSetup)
  {
    delay(1000);
    return;
  }
  readConsole();
  delay(10);
}
