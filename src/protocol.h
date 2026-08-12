#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace protocol
{
  constexpr std::size_t MAX_APPLICATION_FRAME_SIZE = 24;
  constexpr std::size_t UPLINK_FRAME_SIZE = 11;
  constexpr std::size_t MAX_PENDING_TRANSACTIONS = 16;

  enum class PacketHeader : uint8_t
  {
    CommandReceive = 0xA0,
    LiftoffDetection = 0xA1,
    EngineBurn = 0xA2,
    Control = 0xA3,
    Descent = 0xA4,
    RecoveryBeacon = 0xA5,
    RecoveryLogData = 0xA6,
    CommandResult = 0xB0,
    GroundTimeRequest = 0xB1,
  };

  enum class DecodeError : uint8_t
  {
    None,
    UnknownHeader,
    WrongLength,
    ChecksumMismatch,
    NonZeroPadding,
    InvalidField,
  };

  enum class UplinkKind : uint8_t
  {
    MissionGeneric = 0,
    ActuatorEmergency = 1,
    LiftoffDetectionEmergency = 2,
    ComBoardLocal = 3,
    GroundTimeResponse = 4,
  };

  enum class CommandPhase : uint8_t
  {
    Accepted = 0,
    Completed = 1,
    Rejected = 2,
    Failed = 3,
  };

  enum class CommandReason : uint8_t
  {
    None = 0,
    Busy = 1,
    InvalidState = 2,
    InvalidArgument = 3,
    NotConfigured = 4,
    DeviceUnavailable = 5,
    Timeout = 6,
    Stall = 7,
    ProtocolError = 8,
    InterruptedByEmergency = 9,
    PersistenceError = 10,
    InternalError = 11,
    NotSupported = 12,
    SafetyInterlock = 13,
    AlreadySatisfied = 14,
  };

  struct FlightTelemetry
  {
    uint16_t status;
    uint16_t roll;
    uint16_t roll_rate;
    uint8_t tilt_magnitude;
    uint16_t tilt_direction;
    uint8_t fin_angle;
    uint16_t fin_rate;
    uint16_t pressure;
    uint8_t temperature;
    uint8_t airspeed;
    uint16_t requested_torque;
    uint8_t elapsed;
    uint16_t east;
    uint16_t north;
    uint16_t height;
  };

  struct CommandReceiveTelemetry
  {
    uint32_t status;
    uint8_t motor_profile;
    uint8_t tilt_magnitude;
    uint16_t tilt_direction;
    uint8_t fin_mode;
    uint8_t para_mode;
    uint8_t fin_angle;
    uint8_t para_angle;
    uint16_t pressure;
    uint8_t temperature;
    uint8_t airspeed;
    uint8_t logic_voltage;
    uint8_t motor_voltage;
    uint16_t east;
    uint16_t north;
    uint16_t height;
  };

  struct DescentTelemetry
  {
    uint16_t status;
    uint16_t pressure;
    uint8_t temperature;
    uint8_t para_angle;
    uint16_t elapsed;
    uint16_t east;
    uint16_t north;
    uint16_t height;
  };

  struct RecoveryBeacon
  {
    uint8_t logic_voltage;
    uint8_t motor_voltage;
    uint16_t east;
    uint16_t north;
    uint16_t height;
    uint16_t elapsed;
  };

  struct RecoveryLogData
  {
    uint8_t transfer_id;
    uint8_t meta;
    uint32_t offset;
    uint8_t data_length;
    std::array<uint8_t, 16> data;
  };

  struct CommandResult
  {
    uint8_t transaction_id;
    uint8_t command;
    uint8_t phase;
    uint8_t reason;
    uint32_t detail;
  };

  struct GroundTimeRequest
  {
    uint8_t request_id;
  };

  struct DecodedPacket
  {
    PacketHeader header;
    FlightTelemetry flight{};
    CommandReceiveTelemetry command_receive{};
    DescentTelemetry descent{};
    RecoveryBeacon recovery{};
    RecoveryLogData recovery_log{};
    CommandResult command_result{};
    GroundTimeRequest time_request{};
  };

  struct UplinkFrame
  {
    UplinkKind kind;
    uint8_t transaction_id;
    uint8_t command;
    std::array<uint8_t, 6> args;
  };

  struct SemanticValue
  {
    bool numeric;
    int32_t count;
    const char *status;
  };

  std::size_t expectedApplicationLength(uint8_t header);
  uint8_t xorChecksum(const uint8_t *bytes, std::size_t length);
  bool decodeApplicationFrame(
      const uint8_t *frame,
      std::size_t length,
      DecodedPacket &packet,
      DecodeError &error);
  bool encodeUplink(
      const UplinkFrame &frame,
      std::array<uint8_t, UPLINK_FRAME_SIZE> &bytes);

  int32_t signExtend(uint32_t raw, uint8_t bits);
  SemanticValue decodeRoll(uint16_t raw);
  SemanticValue decodeRollRate(uint16_t raw);
  SemanticValue decodeTiltMagnitude(uint8_t raw);
  SemanticValue decodeFinAngle(uint8_t raw);
  SemanticValue decodeFinRate(uint16_t raw);
  SemanticValue decodeRequestedTorque(uint16_t raw);
  SemanticValue decodePressure(uint16_t raw);
  SemanticValue decodeTemperature(uint8_t raw);
  SemanticValue decodeAirspeed(uint8_t raw);
  SemanticValue decodeFlightElapsed(uint8_t raw);
  SemanticValue decodeGnssCoordinate(uint16_t raw);
  SemanticValue decodeGnssHeight(uint16_t raw);
  SemanticValue decodeParaAngle(uint8_t raw);
  SemanticValue decodeLongElapsed(uint16_t raw);
  SemanticValue decodeBattery(uint8_t raw);
  const char *phaseName(uint8_t phase);
  const char *reasonName(uint8_t reason);
  const char *decodeErrorName(DecodeError error);

  class TransactionTracker
  {
  public:
    TransactionTracker();
    bool reserve(UplinkKind kind, uint8_t command, uint8_t &transaction_id);
    bool markResult(const CommandResult &result);
    bool release(uint8_t transaction_id);
    bool isPending(uint8_t transaction_id) const;

  private:
    struct Entry
    {
      bool used;
      uint8_t id;
      UplinkKind kind;
      uint8_t command;
    };

    std::array<Entry, MAX_PENDING_TRANSACTIONS> entries_;
    uint8_t next_id_;
  };
} // protocol名前空間
