#include "protocol.h"

namespace protocol
{
  namespace
  {
    class BitReader
    {
    public:
      BitReader(const uint8_t *bytes, std::size_t length)
          : bytes_(bytes), bit_length_(length * 8), bit_offset_(0) {}

      bool read(uint8_t bits, uint32_t &value)
      {
        if (bits > 32 || bit_offset_ + bits > bit_length_)
        {
          return false;
        }

        value = 0;
        for (uint8_t bit = 0; bit < bits; ++bit)
        {
          const std::size_t source = bit_offset_ + bit;
          value |= static_cast<uint32_t>(
                       (bytes_[source / 8] >> (source % 8)) & 1U)
                   << bit;
        }
        bit_offset_ += bits;
        return true;
      }

    private:
      const uint8_t *bytes_;
      std::size_t bit_length_;
      std::size_t bit_offset_;
    };

    bool read(BitReader &reader, uint8_t bits, uint8_t &value)
    {
      uint32_t raw = 0;
      if (!reader.read(bits, raw))
      {
        return false;
      }
      value = static_cast<uint8_t>(raw);
      return true;
    }

    bool read(BitReader &reader, uint8_t bits, uint16_t &value)
    {
      uint32_t raw = 0;
      if (!reader.read(bits, raw))
      {
        return false;
      }
      value = static_cast<uint16_t>(raw);
      return true;
    }

    bool read(BitReader &reader, uint8_t bits, uint32_t &value)
    {
      return reader.read(bits, value);
    }

    SemanticValue numeric(int32_t count)
    {
      return {true, count, "VALUE"};
    }

    SemanticValue error(const char *status)
    {
      return {false, 0, status};
    }

    SemanticValue commonTime(uint8_t code)
    {
      static const char *const names[] = {
          "PRE_LIFTOFF", "UNAVAILABLE", "RECOVERY_IN_PROGRESS",
          "RTC_RECOVERY_FAILED", "CHECKPOINT_MISSING", "CHECKPOINT_INVALID",
          "GNSS_TIME_UNAVAILABLE", "GROUND_TIME_UNAVAILABLE",
          "ABSOLUTE_TIME_UNAVAILABLE", "TIME_INCONSISTENT", "STALE",
          "OVERFLOW", "PERSISTENCE_ERROR", "POWER_ON_RESET_UNRECOVERABLE",
          "INTERNAL_ERROR", "UNKNOWN"};
      return error(names[code & 0x0F]);
    }

    bool decodeFlight(BitReader &reader, FlightTelemetry &value)
    {
      uint8_t header = 0;
      return read(reader, 8, header) &&
             read(reader, 16, value.status) &&
             read(reader, 16, value.roll) &&
             read(reader, 16, value.roll_rate) &&
             read(reader, 7, value.tilt_magnitude) &&
             read(reader, 9, value.tilt_direction) &&
             read(reader, 8, value.fin_angle) &&
             read(reader, 16, value.fin_rate) &&
             read(reader, 11, value.pressure) &&
             read(reader, 8, value.temperature) &&
             read(reader, 8, value.airspeed) &&
             read(reader, 12, value.requested_torque) &&
             read(reader, 8, value.elapsed) &&
             read(reader, 16, value.east) &&
             read(reader, 16, value.north) &&
             read(reader, 9, value.height);
    }

    bool decodeCommandReceive(BitReader &reader, CommandReceiveTelemetry &value)
    {
      uint8_t header = 0;
      const bool valid = read(reader, 8, header) &&
                         read(reader, 24, value.status) &&
                         read(reader, 8, value.motor_profile) &&
                         read(reader, 7, value.tilt_magnitude) &&
                         read(reader, 9, value.tilt_direction) &&
                         read(reader, 4, value.fin_mode) &&
                         read(reader, 4, value.para_mode) &&
                         read(reader, 8, value.fin_angle) &&
                         read(reader, 8, value.para_angle) &&
                         read(reader, 11, value.pressure) &&
                         read(reader, 8, value.temperature) &&
                         read(reader, 8, value.airspeed) &&
                         read(reader, 8, value.logic_voltage) &&
                         read(reader, 8, value.motor_voltage) &&
                         read(reader, 16, value.east) &&
                         read(reader, 16, value.north) &&
                         read(reader, 9, value.height);
      if (valid)
      {
        if (value.fin_mode > 5 && value.fin_mode < 15)
          value.fin_mode = 15;
        if (value.para_mode > 5 && value.para_mode < 15)
          value.para_mode = 15;
      }
      return valid;
    }

    bool decodeDescent(BitReader &reader, DescentTelemetry &value, uint8_t &padding)
    {
      uint8_t header = 0;
      return read(reader, 8, header) &&
             read(reader, 13, value.status) &&
             read(reader, 11, value.pressure) &&
             read(reader, 8, value.temperature) &&
             read(reader, 8, value.para_angle) &&
             read(reader, 16, value.elapsed) &&
             read(reader, 16, value.east) &&
             read(reader, 16, value.north) &&
             read(reader, 9, value.height) &&
             read(reader, 7, padding);
    }

    bool decodeRecovery(BitReader &reader, RecoveryBeacon &value, uint8_t &padding)
    {
      uint8_t header = 0;
      return read(reader, 8, header) &&
             read(reader, 8, value.logic_voltage) &&
             read(reader, 8, value.motor_voltage) &&
             read(reader, 16, value.east) &&
             read(reader, 16, value.north) &&
             read(reader, 9, value.height) &&
             read(reader, 16, value.elapsed) &&
             read(reader, 7, padding);
    }
  } // 無名名前空間

  std::size_t expectedApplicationLength(uint8_t header)
  {
    switch (static_cast<PacketHeader>(header))
    {
    case PacketHeader::CommandReceive:
      return 22;
    case PacketHeader::LiftoffDetection:
    case PacketHeader::EngineBurn:
    case PacketHeader::Control:
      return 24;
    case PacketHeader::Descent:
      return 15;
    case PacketHeader::RecoveryBeacon:
      return 12;
    case PacketHeader::RecoveryLogData:
      return 24;
    case PacketHeader::CommandResult:
      return 10;
    case PacketHeader::GroundTimeRequest:
      return 3;
    default:
      return 0;
    }
  }

  uint8_t xorChecksum(const uint8_t *bytes, std::size_t length)
  {
    uint8_t checksum = 0;
    for (std::size_t index = 0; index < length; ++index)
    {
      checksum ^= bytes[index];
    }
    return checksum;
  }

  bool decodeApplicationFrame(
      const uint8_t *frame,
      std::size_t length,
      DecodedPacket &packet,
      DecodeError &decode_error)
  {
    decode_error = DecodeError::None;
    if (frame == nullptr || length == 0 || expectedApplicationLength(frame[0]) == 0)
    {
      decode_error = DecodeError::UnknownHeader;
      return false;
    }
    if (length != expectedApplicationLength(frame[0]))
    {
      decode_error = DecodeError::WrongLength;
      return false;
    }
    if (xorChecksum(frame, length - 1) != frame[length - 1])
    {
      decode_error = DecodeError::ChecksumMismatch;
      return false;
    }

    packet = {};
    packet.header = static_cast<PacketHeader>(frame[0]);
    BitReader reader(frame, length - 1);
    uint8_t padding = 0;
    bool valid = false;

    switch (packet.header)
    {
    case PacketHeader::CommandReceive:
      valid = decodeCommandReceive(reader, packet.command_receive) &&
              read(reader, 4, padding);
      break;
    case PacketHeader::LiftoffDetection:
    case PacketHeader::EngineBurn:
    case PacketHeader::Control:
      valid = decodeFlight(reader, packet.flight);
      break;
    case PacketHeader::Descent:
      valid = decodeDescent(reader, packet.descent, padding);
      break;
    case PacketHeader::RecoveryBeacon:
      valid = decodeRecovery(reader, packet.recovery, padding);
      break;
    case PacketHeader::RecoveryLogData:
    {
      uint8_t header = 0;
      valid = read(reader, 8, header) &&
              read(reader, 8, packet.recovery_log.transfer_id) &&
              read(reader, 8, packet.recovery_log.meta) &&
              read(reader, 24, packet.recovery_log.offset) &&
              read(reader, 8, packet.recovery_log.data_length);
      for (std::size_t index = 0; valid && index < packet.recovery_log.data.size(); ++index)
      {
        valid = read(reader, 8, packet.recovery_log.data[index]);
      }
      if (valid && packet.recovery_log.data_length > packet.recovery_log.data.size())
      {
        decode_error = DecodeError::InvalidField;
        return false;
      }
      if (valid && (packet.recovery_log.meta & 0xFC) != 0)
      {
        decode_error = DecodeError::InvalidField;
        return false;
      }
      break;
    }
    case PacketHeader::CommandResult:
    {
      uint8_t header = 0;
      valid = read(reader, 8, header) &&
              read(reader, 8, packet.command_result.transaction_id) &&
              read(reader, 8, packet.command_result.command) &&
              read(reader, 8, packet.command_result.phase) &&
              read(reader, 8, packet.command_result.reason) &&
              read(reader, 32, packet.command_result.detail);
      if (valid &&
          (packet.command_result.transaction_id == 0 ||
           packet.command_result.phase > static_cast<uint8_t>(CommandPhase::Failed) ||
           packet.command_result.reason > static_cast<uint8_t>(CommandReason::AlreadySatisfied)))
      {
        decode_error = DecodeError::InvalidField;
        return false;
      }
      break;
    }
    case PacketHeader::GroundTimeRequest:
    {
      uint8_t header = 0;
      valid = read(reader, 8, header) && read(reader, 8, packet.time_request.request_id);
      if (valid && packet.time_request.request_id == 0)
      {
        decode_error = DecodeError::InvalidField;
        return false;
      }
      break;
    }
    }

    if (!valid)
    {
      decode_error = DecodeError::InvalidField;
      return false;
    }
    if (padding != 0)
    {
      decode_error = DecodeError::NonZeroPadding;
      return false;
    }
    return true;
  }

  bool encodeUplink(
      const UplinkFrame &frame,
      std::array<uint8_t, UPLINK_FRAME_SIZE> &bytes)
  {
    if (frame.transaction_id == 0 || static_cast<uint8_t>(frame.kind) > 4)
    {
      return false;
    }

    bytes = {};
    bytes[0] = 0x55;
    bytes[1] = static_cast<uint8_t>(frame.kind);
    bytes[2] = frame.transaction_id;
    bytes[3] = frame.command;
    for (std::size_t index = 0; index < frame.args.size(); ++index)
    {
      bytes[4 + index] = frame.args[index];
    }
    bytes[10] = xorChecksum(bytes.data(), 10);
    return true;
  }

  int32_t signExtend(uint32_t raw, uint8_t bits)
  {
    const uint32_t mask = (uint32_t{1} << bits) - 1;
    const uint32_t sign = uint32_t{1} << (bits - 1);
    raw &= mask;
    return static_cast<int32_t>((raw ^ sign) - sign);
  }

  SemanticValue decodeRoll(uint16_t raw)
  {
    static const char *const names[] = {
        "UNAVAILABLE", "NOT_INITIALIZED", "SPI_TIMEOUT", "SPI_ERROR",
        "STALE_OR_NO_NEW_SAMPLE", "FIFO_FULL", "FIFO_LOST_PACKET",
        "FIFO_FORMAT_FAULT", "SAMPLE_INVALID", "ODR_CHANGED",
        "SATURATED_OR_OUT_OF_RANGE", "TIMESTAMP_INVALID",
        "ATTITUDE_ESTIMATOR_INVALID", "RESET_INVALIDATED", "INTERNAL_ERROR", "UNKNOWN"};
    return raw >= 0x8000 && raw <= 0x800F ? error(names[raw - 0x8000])
                                          : numeric(signExtend(raw, 16));
  }

  SemanticValue decodeRollRate(uint16_t raw)
  {
    return decodeRoll(raw);
  }

  SemanticValue decodeTiltMagnitude(uint8_t raw)
  {
    static const char *const names[] = {
        "UNAVAILABLE", "NOT_INITIALIZED", "STALE", "ESTIMATOR_INVALID",
        "RESET_INVALIDATED", "OUT_OF_RANGE", "UNKNOWN"};
    if (raw <= 120)
      return numeric(raw);
    return raw <= 127 ? error(names[raw - 121]) : error("INVALID_RAW");
  }

  SemanticValue decodeTiltDirection(uint16_t raw)
  {
    if (raw > 0x01FF)
      return error("INVALID_RAW");
    return raw <= 359 ? numeric(raw) : error("RESERVED");
  }

  SemanticValue decodeFinAngle(uint8_t raw)
  {
    static const char *const names[] = {
        "NOT_INITIALIZED", "SPI_TIMEOUT", "SPI_ERROR", "RESPONSE_PARITY_ERROR",
        "SENSOR_PARITY_ERROR", "INVALID_COMMAND", "FRAMING_ERROR",
        "PIPELINE_STATE_ERROR", "STALE", "UNWRAP_AMBIGUOUS",
        "ZERO_NOT_CONFIGURED", "RESET_INVALIDATED", "OUTPUT_ANGLE_INVALID",
        "OUT_OF_MECHANICAL_RANGE", "INTERNAL_OR_UNKNOWN"};
    return raw <= 240 ? numeric(raw) : error(names[raw - 241]);
  }

  SemanticValue decodeFinRate(uint16_t raw)
  {
    static const char *const names[] = {
        "UNAVAILABLE", "SOURCE_ANGLE_ERROR", "STALE", "NOT_ENOUGH_SAMPLES",
        "UNWRAP_AMBIGUOUS", "TIMESTAMP_INVALID", "ESTIMATOR_NOT_READY",
        "ESTIMATOR_NUMERIC_ERROR", "OUT_OF_RANGE", "RESET_INVALIDATED",
        "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED"};
    return raw >= 0x8000 && raw <= 0x800F ? error(names[raw - 0x8000])
                                          : numeric(signExtend(raw, 16));
  }

  SemanticValue decodeRequestedTorque(uint16_t raw)
  {
    if (raw > 0x0FFF)
      return error("INVALID_RAW");
    static const char *const names[] = {
        "UNAVAILABLE", "CONTROLLER_INPUT_INVALID", "CONTROLLER_NUMERIC_ERROR",
        "RESET_INVALIDATED", "LIMIT_OR_SATURATION_CONFIG_INVALID", "INTERNAL_ERROR",
        "UNKNOWN", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
        "RESERVED", "RESERVED", "RESERVED", "RESERVED"};
    return raw >= 0x800 && raw <= 0x80F ? error(names[raw - 0x800])
                                        : numeric(signExtend(raw, 12));
  }

  SemanticValue decodePressure(uint16_t raw)
  {
    if (raw > 0x07FF)
      return error("INVALID_RAW");
    static const char *const names[] = {
        "NOT_INITIALIZED", "I2C_TIMEOUT_OR_NO_RESPONSE", "I2C_BUS_ERROR",
        "DATA_NOT_READY", "WHO_AM_I_MISMATCH", "RESET_TIMEOUT", "PRESSURE_OVERRUN",
        "STALE", "POWERED_OFF", "BELOW_RANGE", "ABOVE_RANGE", "CONFIGURATION_ERROR",
        "INVALID_SAMPLE", "INTERNAL_ERROR", "UNKNOWN", "UNAVAILABLE"};
    return raw <= 2031 ? numeric(raw) : error(names[raw - 2032]);
  }

  SemanticValue decodeTemperature(uint8_t raw)
  {
    static const char *const names[] = {
        "NOT_INITIALIZED", "I2C_TIMEOUT_OR_NO_RESPONSE", "I2C_BUS_ERROR",
        "DATA_NOT_READY", "WHO_AM_I_MISMATCH", "RESET_TIMEOUT", "TEMPERATURE_OVERRUN",
        "STALE", "POWERED_OFF", "BELOW_RANGE", "ABOVE_RANGE", "CONFIGURATION_ERROR",
        "INVALID_SAMPLE", "INTERNAL_ERROR", "UNKNOWN", "UNAVAILABLE"};
    if (raw <= 200)
      return numeric(raw);
    if (raw < 240)
      return error("RESERVED");
    return error(names[raw - 240]);
  }

  SemanticValue decodeAirspeed(uint8_t raw)
  {
    static const char *const names[] = {
        "BELOW_RANGE_OR_NEGATIVE_DIFFERENTIAL_PRESSURE", "ABOVE_RANGE",
        "STATIC_PRESSURE_INVALID", "SSC_NOT_INITIALIZED", "SSC_I2C_TIMEOUT",
        "SSC_I2C_ERROR", "SSC_STALE", "SSC_COMMAND_MODE", "SSC_DIAGNOSTIC_FAULT",
        "AIRDATA_INTERNAL_INVALID"};
    return raw <= 245 ? numeric(raw) : error(names[raw - 246]);
  }

  SemanticValue decodeFlightElapsed(uint8_t raw)
  {
    return raw <= 0xEF ? numeric(raw) : commonTime(raw);
  }

  SemanticValue decodeGnssCoordinate(uint16_t raw)
  {
    static const char *const names[] = {
        "UNAVAILABLE", "NO_FIX", "STALE", "OUT_OF_RANGE", "INVALID_SAMPLE",
        "RECEIVER_ERROR", "REFERENCE_INVALID", "INTERNAL_ERROR", "UNKNOWN",
        "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED"};
    return raw >= 0x8000 && raw <= 0x800F ? error(names[raw - 0x8000])
                                          : numeric(signExtend(raw, 16));
  }

  SemanticValue decodeGnssHeight(uint16_t raw)
  {
    if (raw > 0x01FF)
      return error("INVALID_RAW");
    static const char *const names[] = {
        "UNAVAILABLE", "NO_FIX", "STALE", "OUT_OF_RANGE", "INVALID_SAMPLE",
        "RECEIVER_ERROR", "INTERNAL_ERROR", "UNKNOWN", "RESERVED", "RESERVED",
        "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED"};
    return raw <= 495 ? numeric(raw) : error(names[raw - 496]);
  }

  SemanticValue decodeParaAngle(uint8_t raw)
  {
    static const char *const names[] = {
        "NOT_INITIALIZED", "UART_TIMEOUT", "UART_PROTOCOL_ERROR", "DEVICE_ERROR_RESPONSE",
        "CONFIGURATION_INVALID", "WRONG_OPERATING_MODE", "STALE", "POSITION_OUT_OF_RANGE",
        "POWERED_OFF", "OPEN_COMMAND_FAILED", "RETRY_EXHAUSTED", "POSITION_INVALID",
        "INTERNAL_ERROR", "UNKNOWN", "UNAVAILABLE"};
    return raw <= 240 ? numeric(raw) : error(names[raw - 241]);
  }

  SemanticValue decodeLongElapsed(uint16_t raw)
  {
    return raw <= 0xFFEF ? numeric(raw) : commonTime(static_cast<uint8_t>(raw));
  }

  SemanticValue decodeBattery(uint8_t raw)
  {
    if (raw <= 240)
      return numeric(raw);
    if (raw <= 252)
      return error("RESERVED");
    if (raw == 253)
      return error("STALE");
    if (raw == 254)
      return error("ADC_ERROR");
    return error("UNAVAILABLE");
  }

  const char *phaseName(uint8_t phase)
  {
    static const char *const names[] = {"Accepted", "Completed", "Rejected", "Failed"};
    return phase < 4 ? names[phase] : "UnknownPhase";
  }

  const char *reasonName(uint8_t reason)
  {
    static const char *const names[] = {
        "None", "Busy", "InvalidState", "InvalidArgument", "NotConfigured",
        "DeviceUnavailable", "Timeout", "Stall", "ProtocolError",
        "InterruptedByEmergency", "PersistenceError", "InternalError",
        "NotSupported", "SafetyInterlock", "AlreadySatisfied"};
    return reason < 15 ? names[reason] : "UnknownReason";
  }

  const char *finModeName(uint8_t mode)
  {
    static const char *const names[] = {
        "Free", "Brake", "PositionHold", "ZeroHold", "RelativeMove", "RollControl"};
    return mode < 6 ? names[mode] : "Unknown";
  }

  const char *paraModeName(uint8_t mode)
  {
    static const char *const names[] = {
        "Free", "Hold", "RelativeMove", "OpeningOrRetrying", "Closing", "PoweredOff"};
    return mode < 6 ? names[mode] : "Unknown";
  }

  const char *decodeErrorName(DecodeError decode_error)
  {
    switch (decode_error)
    {
    case DecodeError::None:
      return "None";
    case DecodeError::UnknownHeader:
      return "UnknownHeader";
    case DecodeError::WrongLength:
      return "WrongLength";
    case DecodeError::ChecksumMismatch:
      return "ChecksumMismatch";
    case DecodeError::NonZeroPadding:
      return "NonZeroPadding";
    case DecodeError::InvalidField:
      return "InvalidField";
    }
    return "Unknown";
  }

  TransactionTracker::TransactionTracker() : entries_{}, next_id_(1) {}

  bool TransactionTracker::reserve(UplinkKind kind, uint8_t command, uint8_t &transaction_id)
  {
    Entry *free_entry = nullptr;
    for (Entry &entry : entries_)
    {
      if (!entry.used && free_entry == nullptr)
      {
        free_entry = &entry;
      }
    }
    if (free_entry == nullptr)
    {
      return false;
    }

    for (uint16_t attempts = 0; attempts < 255; ++attempts)
    {
      const uint8_t candidate = next_id_;
      next_id_ = next_id_ == 255 ? 1 : static_cast<uint8_t>(next_id_ + 1);
      if (!isPending(candidate))
      {
        *free_entry = {true, candidate, kind, command};
        transaction_id = candidate;
        return true;
      }
    }
    return false;
  }

  bool TransactionTracker::markResult(const CommandResult &result)
  {
    for (Entry &entry : entries_)
    {
      if (entry.used && entry.id == result.transaction_id && entry.command == result.command)
      {
        if (result.phase == static_cast<uint8_t>(CommandPhase::Completed) ||
            result.phase == static_cast<uint8_t>(CommandPhase::Rejected) ||
            result.phase == static_cast<uint8_t>(CommandPhase::Failed))
        {
          entry.used = false;
        }
        return true;
      }
    }
    return false;
  }

  bool TransactionTracker::release(uint8_t transaction_id)
  {
    for (Entry &entry : entries_)
    {
      if (entry.used && entry.id == transaction_id)
      {
        entry.used = false;
        return true;
      }
    }
    return false;
  }

  bool TransactionTracker::isPending(uint8_t transaction_id) const
  {
    if (transaction_id == 0)
    {
      return false;
    }
    for (const Entry &entry : entries_)
    {
      if (entry.used && entry.id == transaction_id)
      {
        return true;
      }
    }
    return false;
  }
} // protocol名前空間
