#include "protocol.h"

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

  void testGoldenPackets(const std::map<std::string, std::vector<uint8_t>> &vectors)
  {
    const auto flight = decode(vectors, "LORA_FLIGHT");
    assert(flight.header == protocol::PacketHeader::Control);
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

    const auto command_receive = decode(vectors, "LORA_COMMAND_RECEIVE");
    assert(command_receive.header == protocol::PacketHeader::CommandReceive);
    assert(command_receive.command_receive.status == 0xABCDEF);
    assert(command_receive.command_receive.motor_profile == 3);

    const auto descent = decode(vectors, "LORA_DESCENT");
    assert(descent.header == protocol::PacketHeader::Descent);
    assert(descent.descent.status == 0x1A55);
    assert(descent.descent.pressure == 0x07F7);

    const auto recovery = decode(vectors, "LORA_RECOVERY");
    assert(recovery.header == protocol::PacketHeader::RecoveryBeacon);
    assert(recovery.recovery.logic_voltage == 0xA0);
    assert(recovery.recovery.motor_voltage == 0xF0);

    const auto log = decode(vectors, "LORA_LOG_DATA");
    assert(log.recovery_log.transfer_id == 0x34);
    assert(log.recovery_log.meta == 0x03);
    assert(log.recovery_log.offset == 0x012345);
    assert(log.recovery_log.data_length == 3);
    assert(log.recovery_log.data[0] == 0xDE);
    assert(log.recovery_log.data[1] == 0xAD);
    assert(log.recovery_log.data[2] == 0xBE);

    const auto result = decode(vectors, "LORA_COMMAND_RESULT");
    assert(result.command_result.transaction_id == 0x2A);
    assert(result.command_result.command == 0x13);
    assert(result.command_result.phase == 3);
    assert(result.command_result.reason == 9);
    assert(result.command_result.detail == 0x12345678);

    const auto time_request = decode(vectors, "LORA_TIME_REQUEST");
    assert(time_request.time_request.request_id == 7);
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
    assert(error == protocol::DecodeError::InvalidField);

    bad_result = vectors.at("LORA_COMMAND_RESULT");
    bad_result[7] = 15;
    bad_result.back() = protocol::xorChecksum(bad_result.data() + 3, bad_result.size() - 4);
    assert(!protocol::decodeApplicationFrame(
        bad_result.data() + 3, bad_result.size() - 3, packet, error));
    assert(error == protocol::DecodeError::InvalidField);

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
    assert(protocol::decodeRoll(raw16(vectors, "SCALAR_ROLL_NEG1")).count == -2);
    assert(protocol::decodeRoll(raw16(vectors, "SCALAR_ROLL_MIN")).count == -32752);
    assert(std::string(protocol::decodeRoll(
                           raw16(vectors, "SCALAR_ROLL_RESET_INVALIDATED"))
                           .status) == "RESET_INVALIDATED");

    const uint16_t tilt = raw16(vectors, "SCALAR_TILT_MAX");
    assert(protocol::decodeTiltMagnitude(static_cast<uint8_t>(tilt & 0x7F)).count == 120);
    assert((tilt >> 7) == 359);
    assert(protocol::decodeTiltMagnitude(128).numeric == false);
    assert(std::string(protocol::decodeTiltMagnitude(128).status) == "INVALID_RAW");

    assert(protocol::decodeFinAngle(raw8(vectors, "SCALAR_FIN_ANGLE_MIN")).count == 0);
    assert(protocol::decodeFinAngle(raw8(vectors, "SCALAR_FIN_ANGLE_ZERO")).count == 120);
    assert(protocol::decodeFinAngle(raw8(vectors, "SCALAR_FIN_ANGLE_MAX")).count == 240);
    assert(protocol::decodeFinRate(raw16(vectors, "SCALAR_FIN_RATE_NEG1")).count == -50);
    assert(protocol::decodeRequestedTorque(
               raw16(vectors, "SCALAR_TORQUE_NEG_0P246"))
               .count == -123);
    assert(protocol::decodeRequestedTorque(0x1F85).numeric == false);

    assert(protocol::decodePressure(
               raw16(vectors, "SCALAR_LPS_PRESSURE_1013P2"))
               .count == 1066);
    assert(protocol::decodePressure(raw16(vectors, "SCALAR_LPS_PRESSURE_MAX")).count == 2031);
    assert(std::string(protocol::decodePressure(
                           raw16(vectors, "SCALAR_LPS_PRESSURE_STALE"))
                           .status) == "STALE");
    assert(protocol::decodePressure(0x0FFF).numeric == false);
    assert(protocol::decodeTemperature(raw8(vectors, "SCALAR_LPS_TEMP_20")).count == 70);

    assert(protocol::decodeAirspeed(raw8(vectors, "SCALAR_AIRSPEED_245")).count == 245);
    assert(std::string(protocol::decodeAirspeed(
                           raw8(vectors, "SCALAR_AIRSPEED_NEGATIVE"))
                           .status) == "BELOW_RANGE_OR_NEGATIVE_DIFFERENTIAL_PRESSURE");
    assert(protocol::decodeGnssHeight(0xFFFF).numeric == false);
    assert(std::string(protocol::decodeAirspeed(
                           raw8(vectors, "SCALAR_AIRSPEED_STALE"))
                           .status) == "SSC_STALE");

    assert(protocol::decodeFlightElapsed(
               raw8(vectors, "SCALAR_FLIGHT_ELAPSED_23P9"))
               .count == 239);
    assert(std::string(protocol::decodeFlightElapsed(
                           raw8(vectors, "SCALAR_FLIGHT_ELAPSED_STALE"))
                           .status) == "STALE");

    assert(protocol::decodeGnssCoordinate(
               raw16(vectors, "SCALAR_GNSS_EAST_NEG1"))
               .count == -1);
    assert(std::string(protocol::decodeGnssCoordinate(
                           raw16(vectors, "SCALAR_GNSS_NO_FIX"))
                           .status) == "NO_FIX");
    assert(std::string(protocol::decodeGnssCoordinate(
                           raw16(vectors, "SCALAR_GNSS_STALE"))
                           .status) == "STALE");

    assert(protocol::decodeGnssHeight(
               raw16(vectors, "SCALAR_GNSS_HEIGHT_100"))
               .count == 40);
    assert(std::string(protocol::decodeGnssHeight(
                           raw16(vectors, "SCALAR_GNSS_HEIGHT_NO_FIX"))
                           .status) == "NO_FIX");
    assert(std::string(protocol::decodeGnssHeight(
                           raw16(vectors, "SCALAR_GNSS_HEIGHT_STALE"))
                           .status) == "STALE");

    assert(protocol::decodeParaAngle(raw8(vectors, "SCALAR_PARA_ANGLE_360")).count == 240);
    assert(std::string(protocol::decodeParaAngle(
                           raw8(vectors, "SCALAR_PARA_STALE"))
                           .status) == "STALE");
    assert(protocol::decodeLongElapsed(
               raw16(vectors, "SCALAR_DESCENT_ELAPSED_MAX"))
               .count == 0xFFEF);
    assert(std::string(protocol::decodeLongElapsed(
                           raw16(vectors, "SCALAR_DESCENT_ELAPSED_STALE"))
                           .status) == "STALE");

    assert(protocol::decodeBattery(raw8(vectors, "SCALAR_BATTERY_12V")).count == 240);
    assert(std::string(protocol::decodeBattery(
                           raw8(vectors, "SCALAR_BATTERY_FIRST_RESERVED"))
                           .status) == "RESERVED");
    assert(std::string(protocol::decodeBattery(
                           raw8(vectors, "SCALAR_BATTERY_STALE"))
                           .status) == "STALE");
  }

  void testTransactionTracker()
  {
    protocol::TransactionTracker tracker;
    std::array<uint8_t, protocol::MAX_PENDING_TRANSACTIONS> ids{};
    for (uint8_t &id : ids)
    {
      assert(tracker.reserve(protocol::UplinkKind::MissionGeneric, 0x13, id));
      assert(id != 0);
      assert(tracker.isPending(id));
    }
    uint8_t extra = 0;
    assert(!tracker.reserve(protocol::UplinkKind::MissionGeneric, 0x13, extra));

    const protocol::CommandResult accepted{ids[0], 0x13, 0, 0, 0};
    assert(tracker.markResult(accepted));
    assert(tracker.isPending(ids[0]));
    const protocol::CommandResult completed{ids[0], 0x13, 1, 0, 0};
    assert(tracker.markResult(completed));
    assert(!tracker.isPending(ids[0]));
  }
} // 無名名前空間

int main()
{
  const auto vectors = loadVectors();
  testGoldenPackets(vectors);
  testGoldenUplinks(vectors);
  testMalformedFrames(vectors);
  testScalarSemantics(vectors);
  testTransactionTracker();
  return 0;
}
