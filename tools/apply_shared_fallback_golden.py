from pathlib import Path

path = Path("test/host_protocol/test_main.cpp")
text = path.read_text(encoding="utf-8")
old = """void testMissionLinkFallbackHeaderMigration() {
  USBV1Decoder decoder;

"""
new = """void testMissionLinkFallbackHeaderMigration() {
  USBV1Decoder decoder;

  const auto shared =
      hexToBytes(goldenValue(\"LORA_MISSION_LINK_FALLBACK\"));
  assert(shared.size() == 27);
  assert(shared[0] == 0x00 && shared[1] == 0x00 && shared[2] == 0x04);
  const std::vector<uint8_t> shared_app(shared.begin() + 3, shared.end());
  const auto shared_decoded = decoder.decode(makeRxLine(shared_app, -84));
  assert(std::holds_alternative<usbv1::MissionLinkFallbackTelemetry>(
      shared_decoded.payload));

"""
if text.count(old) != 1:
    raise SystemExit("shared fallback host-test anchor was not unique")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
