#include "protocol.h"
#include "uplink_boundary_policy.h"

#include <array>
#include <cassert>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace
{
  std::vector<uint8_t> fromHex(const std::string &hex)
  {
    assert(hex.size() % 2 == 0);
    std::vector<uint8_t> bytes;
    bytes.reserve(hex.size() / 2);
    for (std::size_t index = 0; index < hex.size(); index += 2)
    {
      bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(index, 2), nullptr, 16)));
    }
    return bytes;
  }

  std::map<std::string, std::vector<uint8_t>> loadVectors()
  {
    std::ifstream input("testdata/99l_protocol_golden_vectors.txt");
    assert(input.good());

    std::map<std::string, std::vector<uint8_t>> vectors;
    std::string line;
    while (std::getline(input, line))
    {
      if (line.empty() || line[0] == '#')
        continue;
      const std::size_t separator = line.find('=');
      assert(separator != std::string::npos);
      vectors.emplace(line.substr(0, separator), fromHex(line.substr(separator + 1)));
    }
    return vectors;
  }

  protocol::DecodedPacket decode(
      const std::map<std::string, std::vector<uint8_t>> &vectors,
      const std::string &name)
  {
    const std::vector<uint8_t> &wire = vectors.at(name);
    assert(wire.size() >= 4);
    protocol::DecodedPacket packet{};
    protocol::DecodeError error = protocol::DecodeError::None;
    assert(protocol::decodeApplicationFrame(
        wire.data() + 3, wire.size() - 3, packet, error));
    return packet;
  }

  protocol::DecodedPacket decodeFlightVariant(
      const std::map<std::string, std::vector<uint8_t>> &vectors,
      protocol::PacketHeader header)
  {
    std::vector<uint8_t> wire = vectors.at("LORA_FLIGHT");
    wire[3] = static_cast<uint8_t>(header);
    wire.back() = protocol::xorChecksum(wire.data() + 3, wire.size() - 4);
    protocol::DecodedPacket packet{};
    protocol::DecodeError error = protocol::DecodeError::None;
    assert(protocol::decodeApplicationFrame(
        wire.data() + 3, wire.size() - 3, packet, error));
    return packet;
  }

  void assertNumeric(protocol::SemanticValue value, int32_t count)
  {
    assert(value.numeric);
    assert(value.count == count);
  }

  void assertStatus(protocol::SemanticValue value, const char *status)
  {
    assert(!value.numeric);
    assert(std::string(value.status) == status);
  }

  uint8_t raw8(
      const std::map<std::string, std::vector<uint8_t>> &vectors,
      const std::string &name)
  {
    const auto &bytes = vectors.at(name);
    assert(bytes.size() == 1);
    return bytes[0];
  }

  uint16_t raw16(
      const std::map<std::string, std::vector<uint8_t>> &vectors,
      const std::string &name)
  {
    const auto &bytes = vectors.at(name);
    assert(bytes.size() == 2);
    return static_cast<uint16_t>(bytes[0]) |
           static_cast<uint16_t>(bytes[1]) << 8;
  }

  void assertFlightFields(
      const protocol::DecodedPacket &flight,
      protocol::PacketHeader header)
  {
    assert(flight.header == header);
    assert(flight.flight.status == 0xA55A);
    assert(flight.flight.roll == 0xFFFE);
    assert(flight.flight.roll_rate == 0x04D2);
    assert(flight.flight.tilt_magnitude == 20);
    assert(flight.flight.tilt_direction == 280);
    assert(flight.flight.fin_angle == 0x78);
    assert(flight.flight.fin_rate == 0xFFCE);
    assert(flight.flight.pressure == 0x042A);
    assert(flight.flight.temperature == 0x46);
    assert(flight.flight.airspeed == 0x3D);
    assert(flight.flight.requested_torque == 0x0F85);
    assert(flight.flight.elapsed == 0x7B);
    assert(flight.flight.east == 0xFFF4);
    assert(flight.flight.north == 0x0022);
    assert(flight.flight.height == 0x0028);
  }

  void testGoldenPackets(const std::map<std::string, std::vector<uint8_t>> &vectors)
  {
    assertFlightFields(
        decodeFlightVariant(vectors, protocol::PacketHeader::LiftoffDetection),
        protocol::PacketHeader::LiftoffDetection);
    assertFlightFields(
        decodeFlightVariant(vectors, protocol::PacketHeader::EngineBurn),
        protocol::PacketHeader::EngineBurn);
    assertFlightFields(decode(vectors, "LORA_FLIGHT"), protocol::PacketHeader::Control);

    const auto command_receive = decode(vectors, "LORA_COMMAND_RECEIVE");
    assert(command_receive.header == protocol::PacketHeader::CommandReceive);
    assert(command_receive.command_receive.status == 0xABCDEF);
    assert(command_receive.command_receive.motor_profile == 3);
    assert(command_receive.command_receive.tilt_magnitude == 27);
    assert(command_receive.command_receive.tilt_direction == 281);
    assert(command_receive.command_receive.fin_mode == 3);
    assert(command_receive.command_receive.para_mode == 1);
    assert(command_receive.command_receive.fin_angle == 120);
    assert(command_receive.command_receive.para_angle == 60);
    assert(command_receive.command_receive.pressure == 1066);
    assert(command_receive.command_receive.temperature == 70);
    assert(command_receive.command_receive.airspeed == 249);
    assert(command_receive.command_receive.logic_voltage == 160);
    assert(command_receive.command_receive.motor_voltage == 220);
    assert(command_receive.command_receive.east == 0x8001);
    assert(command_receive.command_receive.north == 0x8001);
    assert(command_receive.command_receive.height == 497);

    const auto descent = decode(vectors, "LORA_DESCENT");
    assert(descent.header == protocol::PacketHeader::Descent);
    assert(descent.descent.status == 0x1A55);
    assert(((descent.descent.status >> 2U) & 0x03U) == 1);
    assert(descent.descent.pressure == 0x07F7);
    assert(descent.descent.temperature == 0xF7);
    assert(descent.descent.para_angle == 0xF7);
    assert(descent.descent.elapsed == 0xFFFA);
    assert(descent.descent.east == 0x8002);
    assert(descent.descent.north == 0xFFFF);
    assert(descent.descent.height == 0x01F2);

    const auto recovery = decode(vectors, "LORA_RECOVERY");
    assert(recovery.header == protocol::PacketHeader::RecoveryBeacon);
    assert(recovery.recovery.logic_voltage == 0xA0);
    assert(recovery.recovery.motor_voltage == 0xF0);
    assert(recovery.recovery.east == 100);
    assert(recovery.recovery.north == 0xFF9C);
    assert(recovery.recovery.height == 60);
    assert(recovery.recovery.elapsed == 12);

    const auto log = decode(vectors, "LORA_LOG_DATA");
    assert(log.header == protocol::PacketHeader::RecoveryLogData);
    assert(log.recovery_log.transfer_id == 0x34);
    assert(log.recovery_log.meta == 0x03);
    assert(log.recovery_log.offset == 0x012345);
    assert(log.recovery_log.data_length == 3);
    assert(log.recovery_log.data[0] == 0xDE);
    assert(log.recovery_log.data[1] == 0xAD);
    assert(log.recovery_log.data[2] == 0xBE);
    for (std::size_t index = 3; index < log.recovery_log.data.size(); ++index)
      assert(log.recovery_log.data[index] == 0);

    const auto result = decode(vectors, "LORA_COMMAND_RESULT");
    assert(result.header == protocol::PacketHeader::CommandResult);
    assert(result.command_result.transaction_id == 0x2A);
    assert(result.command_result.command == 0x13);
    assert(result.command_result.phase == 3);
    assert(result.command_result.reason == 9);
    assert(result.command_result.detail == 0x12345678);

    const auto time_request = decode(vectors, "LORA_TIME_REQUEST");
    assert(time_request.header == protocol::PacketHeader::GroundTimeRequest);
    assert(time_request.time_request.request_id == 7);
  }

  void testReservedModes(const std::map<std::string, std::vector<uint8_t>> &vectors)
  {
    std::vector<uint8_t> wire = vectors.at("LORA_COMMAND_RECEIVE");
    wire[10] = 0xA6;
    wire.back() = protocol::xorChecksum(wire.data() + 3, wire.size() - 4);
    protocol::DecodedPacket packet{};
    protocol::DecodeError error = protocol::DecodeError::None;
    assert(!protocol::decodeApplicationFrame(
        wire.data() + 3, wire.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidEnum);

    wire = vectors.at("LORA_COMMAND_RECEIVE");
    wire[10] = 0xFF;
    wire.back() = protocol::xorChecksum(wire.data() + 3, wire.size() - 4);
    assert(protocol::decodeApplicationFrame(
        wire.data() + 3, wire.size() - 3, packet, error));
    assert(packet.command_receive.fin_mode == 15);
    assert(packet.command_receive.para_mode == 15);
    assert(std::string(protocol::finModeName(packet.command_receive.fin_mode)) == "Unknown");
    assert(std::string(protocol::paraModeName(packet.command_receive.para_mode)) == "Unknown");
    assert(std::string(protocol::finModeName(5)) == "RollControl");
    assert(std::string(protocol::paraModeName(5)) == "PoweredOff");
  }

  void testGoldenUplinks(const std::map<std::string, std::vector<uint8_t>> &vectors)
  {
    struct Case
    {
      const char *name;
      protocol::UplinkFrame frame;
    };
    const Case cases[] = {
        {"UPLINK_GENERIC", {protocol::UplinkKind::MissionGeneric, 0x2A, 0x13, {0x85, 0xFF, 0, 0, 0, 0}}},
        {"UPLINK_EMERGENCY", {protocol::UplinkKind::ActuatorEmergency, 0x2B, 0, {0, 0, 0, 0, 0, 0}}},
        {"UPLINK_TIME", {protocol::UplinkKind::GroundTimeResponse, 7, 2, {0x78, 0x56, 0x34, 0x12, 0xE7, 0x03}}},
    };

    for (const Case &test_case : cases)
    {
      std::array<uint8_t, protocol::UPLINK_FRAME_SIZE> encoded{};
      assert(protocol::encodeUplink(test_case.frame, encoded));
      const auto &expected = vectors.at(test_case.name);
      assert(expected.size() == encoded.size());
      assert(std::equal(encoded.begin(), encoded.end(), expected.begin(), expected.end()));
    }
  }

  void testMalformedFrames(const std::map<std::string, std::vector<uint8_t>> &vectors)
  {
    protocol::DecodedPacket packet{};
    protocol::DecodeError error = protocol::DecodeError::None;

    auto bad_checksum = vectors.at("LORA_FLIGHT");
    bad_checksum.back() ^= 1;
    assert(!protocol::decodeApplicationFrame(
        bad_checksum.data() + 3, bad_checksum.size() - 3, packet, error));
    assert(error == protocol::DecodeError::ChecksumMismatch);

    auto bad_padding = vectors.at("LORA_DESCENT");
    bad_padding[bad_padding.size() - 2] |= 0x80;
    bad_padding.back() = protocol::xorChecksum(bad_padding.data() + 3, bad_padding.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_padding.data() + 3, bad_padding.size() - 3, packet, error));
    assert(error == protocol::DecodeError::NonZeroPadding);

    auto bad_result = vectors.at("LORA_COMMAND_RESULT");
    bad_result[4] = 0;
    bad_result.back() = protocol::xorChecksum(bad_result.data() + 3, bad_result.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_result.data() + 3, bad_result.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidField);

    bad_result = vectors.at("LORA_COMMAND_RESULT");
    bad_result[6] = 4;
    bad_result.back() = protocol::xorChecksum(bad_result.data() + 3, bad_result.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_result.data() + 3, bad_result.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidEnum);

    bad_result = vectors.at("LORA_COMMAND_RESULT");
    bad_result[7] = 15;
    bad_result.back() = protocol::xorChecksum(bad_result.data() + 3, bad_result.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_result.data() + 3, bad_result.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidEnum);

    auto bad_log = vectors.at("LORA_LOG_DATA");
    bad_log[5] = 0x04;
    bad_log.back() = protocol::xorChecksum(bad_log.data() + 3, bad_log.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_log.data() + 3, bad_log.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidField);

    auto bad_time = vectors.at("LORA_TIME_REQUEST");
    bad_time[4] = 0;
    bad_time.back() = protocol::xorChecksum(bad_time.data() + 3, bad_time.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_time.data() + 3, bad_time.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidField);
  }

  void testScalarSemantics(const std::map<std::string, std::vector<uint8_t>> &vectors)
  {
    assertNumeric(protocol::decodeRoll(0x0000), 0);
    assertNumeric(protocol::decodeRoll(0x7FFF), 32767);
    assertNumeric(protocol::decodeRoll(raw16(vectors, "SCALAR_ROLL_NEG1")), -2);
    assertNumeric(protocol::decodeRoll(raw16(vectors, "SCALAR_ROLL_MIN")), -32752);
    assertStatus(protocol::decodeRoll(0x8000), "UNAVAILABLE");
    assertStatus(protocol::decodeRoll(0x8004), "STALE_OR_NO_NEW_SAMPLE");
    assertStatus(protocol::decodeRoll(
                     raw16(vectors, "SCALAR_ROLL_RESET_INVALIDATED")),
                 "RESET_INVALIDATED");
    assertStatus(protocol::decodeRoll(0x800F), "UNKNOWN");
    assertNumeric(protocol::decodeRollRate(0x7FFF), 32767);
    assertNumeric(protocol::decodeRollRate(0x8010), -32752);
    assertStatus(protocol::decodeRollRate(0x8000), "UNAVAILABLE");
    assertStatus(protocol::decodeRollRate(0x8004), "STALE_OR_NO_NEW_SAMPLE");
    assertStatus(protocol::decodeRollRate(0x800D), "RESET_INVALIDATED");

    const uint16_t tilt = raw16(vectors, "SCALAR_TILT_MAX");
    assertNumeric(protocol::decodeTiltMagnitude(0), 0);
    assertNumeric(protocol::decodeTiltMagnitude(static_cast<uint8_t>(tilt & 0x7F)), 120);
    assertStatus(protocol::decodeTiltMagnitude(121), "UNAVAILABLE");
    assertStatus(protocol::decodeTiltMagnitude(123), "STALE");
    assertStatus(protocol::decodeTiltMagnitude(125), "RESET_INVALIDATED");
    assertStatus(protocol::decodeTiltMagnitude(127), "UNKNOWN");
    assertNumeric(protocol::decodeTiltDirection(0), 0);
    assertNumeric(protocol::decodeTiltDirection(tilt >> 7), 359);
    assertStatus(protocol::decodeTiltDirection(360), "RESERVED");
    assertStatus(protocol::decodeTiltDirection(511), "RESERVED");
    assertStatus(protocol::decodeTiltDirection(512), "INVALID_RAW");
    assert((tilt >> 7) == 359);
    assertStatus(protocol::decodeTiltMagnitude(128), "INVALID_RAW");

    assertNumeric(protocol::decodeFinAngle(raw8(vectors, "SCALAR_FIN_ANGLE_MIN")), 0);
    assertNumeric(protocol::decodeFinAngle(raw8(vectors, "SCALAR_FIN_ANGLE_ZERO")), 120);
    assertNumeric(protocol::decodeFinAngle(raw8(vectors, "SCALAR_FIN_ANGLE_MAX")), 240);
    assertStatus(protocol::decodeFinAngle(241), "NOT_INITIALIZED");
    assertStatus(protocol::decodeFinAngle(249), "STALE");
    assertStatus(protocol::decodeFinAngle(252), "RESET_INVALIDATED");
    assertStatus(protocol::decodeFinAngle(255), "INTERNAL_OR_UNKNOWN");
    assertNumeric(protocol::decodeFinRate(0x7FFF), 32767);
    assertNumeric(protocol::decodeFinRate(0x8010), -32752);
    assertNumeric(protocol::decodeFinRate(raw16(vectors, "SCALAR_FIN_RATE_NEG1")), -50);
    assertStatus(protocol::decodeFinRate(0x8000), "UNAVAILABLE");
    assertStatus(protocol::decodeFinRate(0x8002), "STALE");
    assertStatus(protocol::decodeFinRate(0x8009), "RESET_INVALIDATED");
    assertStatus(protocol::decodeFinRate(0x800F), "RESERVED");
    assertNumeric(protocol::decodeRequestedTorque(0x000), 0);
    assertNumeric(protocol::decodeRequestedTorque(0x7FF), 2047);
    assertNumeric(protocol::decodeRequestedTorque(0x810), -2032);
    assertNumeric(protocol::decodeRequestedTorque(
                      raw16(vectors, "SCALAR_TORQUE_NEG_0P246")),
                  -123);
    assertStatus(protocol::decodeRequestedTorque(0x800), "UNAVAILABLE");
    assertStatus(protocol::decodeRequestedTorque(0x803), "RESET_INVALIDATED");
    assertStatus(protocol::decodeRequestedTorque(0x807), "RESERVED");
    assertStatus(protocol::decodeRequestedTorque(0x1F85), "INVALID_RAW");

    assertNumeric(protocol::decodePressure(0), 0);
    assertNumeric(protocol::decodePressure(
                      raw16(vectors, "SCALAR_LPS_PRESSURE_1013P2")),
                  1066);
    assertNumeric(protocol::decodePressure(
                      raw16(vectors, "SCALAR_LPS_PRESSURE_MAX")),
                  2031);
    assertStatus(protocol::decodePressure(2032), "NOT_INITIALIZED");
    assertStatus(protocol::decodePressure(
                     raw16(vectors, "SCALAR_LPS_PRESSURE_STALE")),
                 "STALE");
    assertStatus(protocol::decodePressure(2046), "UNKNOWN");
    assertStatus(protocol::decodePressure(2047), "UNAVAILABLE");
    assertStatus(protocol::decodePressure(0x0FFF), "INVALID_RAW");
    assertNumeric(protocol::decodeTemperature(0), 0);
    assertNumeric(protocol::decodeTemperature(
                      raw8(vectors, "SCALAR_LPS_TEMP_20")),
                  70);
    assertNumeric(protocol::decodeTemperature(200), 200);
    assertStatus(protocol::decodeTemperature(201), "RESERVED");
    assertStatus(protocol::decodeTemperature(239), "RESERVED");
    assertStatus(protocol::decodeTemperature(240), "NOT_INITIALIZED");
    assertStatus(protocol::decodeTemperature(247), "STALE");
    assertStatus(protocol::decodeTemperature(255), "UNAVAILABLE");

    assertNumeric(protocol::decodeAirspeed(0), 0);
    assertNumeric(protocol::decodeAirspeed(raw8(vectors, "SCALAR_AIRSPEED_245")), 245);
    assertStatus(protocol::decodeAirspeed(
                     raw8(vectors, "SCALAR_AIRSPEED_NEGATIVE")),
                 "BELOW_RANGE_OR_NEGATIVE_DIFFERENTIAL_PRESSURE");
    assertStatus(protocol::decodeAirspeed(249), "SSC_NOT_INITIALIZED");
    assertStatus(protocol::decodeAirspeed(
                     raw8(vectors, "SCALAR_AIRSPEED_STALE")),
                 "SSC_STALE");
    assertStatus(protocol::decodeAirspeed(255), "AIRDATA_INTERNAL_INVALID");

    assertNumeric(protocol::decodeFlightElapsed(0), 0);
    assertNumeric(protocol::decodeFlightElapsed(
                      raw8(vectors, "SCALAR_FLIGHT_ELAPSED_23P9")),
                  239);
    assertStatus(protocol::decodeFlightElapsed(240), "PRE_LIFTOFF");
    assertStatus(protocol::decodeFlightElapsed(241), "UNAVAILABLE");
    assertStatus(protocol::decodeFlightElapsed(
                     raw8(vectors, "SCALAR_FLIGHT_ELAPSED_STALE")),
                 "STALE");
    assertStatus(protocol::decodeFlightElapsed(253), "POWER_ON_RESET_UNRECOVERABLE");
    assertStatus(protocol::decodeFlightElapsed(255), "UNKNOWN");

    assertNumeric(protocol::decodeGnssCoordinate(0), 0);
    assertNumeric(protocol::decodeGnssCoordinate(0x7FFF), 32767);
    assertNumeric(protocol::decodeGnssCoordinate(0x8010), -32752);
    assertNumeric(protocol::decodeGnssCoordinate(
                      raw16(vectors, "SCALAR_GNSS_EAST_NEG1")),
                  -1);
    assertStatus(protocol::decodeGnssCoordinate(0x8000), "UNAVAILABLE");
    assertStatus(protocol::decodeGnssCoordinate(
                     raw16(vectors, "SCALAR_GNSS_NO_FIX")),
                 "NO_FIX");
    assertStatus(protocol::decodeGnssCoordinate(
                     raw16(vectors, "SCALAR_GNSS_STALE")),
                 "STALE");
    assertStatus(protocol::decodeGnssCoordinate(0x8009), "RESERVED");

    assertNumeric(protocol::decodeGnssHeight(0), 0);
    assertNumeric(protocol::decodeGnssHeight(
                      raw16(vectors, "SCALAR_GNSS_HEIGHT_100")),
                  40);
    assertNumeric(protocol::decodeGnssHeight(495), 495);
    assertStatus(protocol::decodeGnssHeight(496), "UNAVAILABLE");
    assertStatus(protocol::decodeGnssHeight(
                     raw16(vectors, "SCALAR_GNSS_HEIGHT_NO_FIX")),
                 "NO_FIX");
    assertStatus(protocol::decodeGnssHeight(
                     raw16(vectors, "SCALAR_GNSS_HEIGHT_STALE")),
                 "STALE");
    assertStatus(protocol::decodeGnssHeight(504), "RESERVED");
    assertStatus(protocol::decodeGnssHeight(0xFFFF), "INVALID_RAW");

    assertNumeric(protocol::decodeParaAngle(0), 0);
    assertNumeric(protocol::decodeParaAngle(
                      raw8(vectors, "SCALAR_PARA_ANGLE_360")),
                  240);
    assertStatus(protocol::decodeParaAngle(241), "NOT_INITIALIZED");
    assertStatus(protocol::decodeParaAngle(
                     raw8(vectors, "SCALAR_PARA_STALE")),
                 "STALE");
    assertStatus(protocol::decodeParaAngle(252), "POSITION_INVALID");
    assertStatus(protocol::decodeParaAngle(255), "UNAVAILABLE");
    assertNumeric(protocol::decodeLongElapsed(0), 0);
    assertNumeric(protocol::decodeLongElapsed(
                      raw16(vectors, "SCALAR_DESCENT_ELAPSED_MAX")),
                  0xFFEF);
    assertStatus(protocol::decodeLongElapsed(0xFFF0), "PRE_LIFTOFF");
    assertStatus(protocol::decodeLongElapsed(0xFFF1), "UNAVAILABLE");
    assertStatus(protocol::decodeLongElapsed(
                     raw16(vectors, "SCALAR_DESCENT_ELAPSED_STALE")),
                 "STALE");
    assertStatus(protocol::decodeLongElapsed(0xFFFD), "POWER_ON_RESET_UNRECOVERABLE");

    assertNumeric(protocol::decodeBattery(0), 0);
    assertNumeric(protocol::decodeBattery(raw8(vectors, "SCALAR_BATTERY_12V")), 240);
    assertStatus(protocol::decodeBattery(
                     raw8(vectors, "SCALAR_BATTERY_FIRST_RESERVED")),
                 "RESERVED");
    assertStatus(protocol::decodeBattery(252), "RESERVED");
    assertStatus(protocol::decodeBattery(
                     raw8(vectors, "SCALAR_BATTERY_STALE")),
                 "STALE");
    assertStatus(protocol::decodeBattery(254), "ADC_ERROR");
    assertStatus(protocol::decodeBattery(255), "UNAVAILABLE");
  }

  void testTransactionTracker()
  {
    protocol::TransactionTracker tracker;
    std::array<uint8_t, protocol::MAX_PENDING_TRANSACTIONS> ids{};
    const std::size_t normal_capacity =
        ids.size() - protocol::EMERGENCY_RESERVED_TRANSACTIONS;
    for (std::size_t index = 0; index < normal_capacity; ++index)
    {
      uint8_t &id = ids[index];
      assert(tracker.reserve(protocol::UplinkKind::MissionGeneric, 0x13, id));
      assert(id != 0);
      assert(tracker.isPending(id));
    }
    uint8_t extra = 0;
    assert(!tracker.reserve(protocol::UplinkKind::MissionGeneric, 0x13, extra));
    assert(tracker.reserve(protocol::UplinkKind::ActuatorEmergency, 0xF0,
                           ids[normal_capacity]));
    assert(tracker.reserve(protocol::UplinkKind::LiftoffDetectionEmergency,
                           0xF1, ids[normal_capacity + 1]));
    assert(!tracker.reserve(protocol::UplinkKind::ActuatorEmergency, 0xF0,
                            extra));

    const protocol::CommandResult accepted{ids[0], 0x13, 0, 0, 0};
    assert(tracker.markResult(accepted));
    assert(tracker.isPending(ids[0]));
    const protocol::CommandResult completed{ids[0], 0x13, 1, 0, 0};
    assert(tracker.markResult(completed));
    assert(!tracker.isPending(ids[0]));
    assert(!tracker.release(ids[0]));
    assert(tracker.release(ids[1]));
    assert(!tracker.isPending(ids[1]));
    const protocol::CommandResult mismatched{ids[2], 0x14, 1, 0, 0};
    assert(!tracker.markResult(mismatched));
  }

  void testUplinkBoundaryPolicy()
  {
    assert(uplink_boundary_policy::isFresh(20000, 0, 20000));
    assert(!uplink_boundary_policy::isFresh(20001, 0, 20000));
    assert(uplink_boundary_policy::elapsedUs(5, UINT32_MAX - 4) == 10);
    assert(uplink_boundary_policy::isFresh(5, UINT32_MAX - 4, 20));
    assert(!uplink_boundary_policy::deadlineExpiredMs(2199, 0, 2200));
    assert(uplink_boundary_policy::deadlineExpiredMs(2200, 0, 2200));
    assert(!uplink_boundary_policy::deadlineExpiredMs(
        5, UINT32_MAX - 4, 11));
    assert(uplink_boundary_policy::deadlineExpiredMs(
        5, UINT32_MAX - 4, 10));
    uint8_t streak = uplink_boundary_policy::resetPeriodicStreak();
    assert(streak == 0);
    streak = uplink_boundary_policy::advancePeriodicStreak(streak);
    assert(!uplink_boundary_policy::periodicModeActive(streak, 3));
    streak = uplink_boundary_policy::advancePeriodicStreak(streak);
    assert(!uplink_boundary_policy::periodicModeActive(streak, 3));
    streak = uplink_boundary_policy::advancePeriodicStreak(streak);
    assert(uplink_boundary_policy::periodicModeActive(streak, 3));
    streak = uplink_boundary_policy::resetPeriodicStreak();
    assert(streak == 0);
    for (uint16_t count = 0; count < 300; ++count)
    {
      streak = uplink_boundary_policy::advancePeriodicStreak(streak);
    }
    assert(streak == UINT8_MAX);
    assert(uplink_boundary_policy::periodicModeActive(streak, 3));
  }
} // 無名名前空間

int main()
{
  const auto vectors = loadVectors();
  testGoldenPackets(vectors);
  testReservedModes(vectors);
  testGoldenUplinks(vectors);
  testMalformedFrames(vectors);
  testScalarSemantics(vectors);
  testTransactionTracker();
  testUplinkBoundaryPolicy();
  return 0;
}
