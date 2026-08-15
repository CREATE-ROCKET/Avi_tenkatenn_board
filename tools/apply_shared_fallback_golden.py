from pathlib import Path

path = Path("test/host_protocol/test_main.cpp")
text = path.read_text(encoding="utf-8")
old = """  void testMissionLinkFallbackHeaderMigration()
  {
    std::array<uint8_t, 24> frame{};
"""
new = """  void testMissionLinkFallbackHeaderMigration()
  {
    const auto vectors = loadVectors();
    const auto shared = decode(vectors, \"LORA_MISSION_LINK_FALLBACK\");
    assert(shared.header == protocol::PacketHeader::MissionLinkFallbackTelemetry);

    std::array<uint8_t, 24> frame{};
"""
if text.count(old) != 1:
    raise SystemExit("shared fallback host-test anchor was not unique")
path.write_text(text.replace(old, new, 1), encoding="utf-8")
