#pragma once

#include <stdint.h>

enum class BootMode
{
  Communication,
  LoRaSetup
};

constexpr BootMode BOOT_MODE = BootMode::Communication;

constexpr uint8_t top_led = 19;
constexpr uint8_t liftoff_led = 21;
constexpr uint8_t control_led = 13;
constexpr uint8_t update_led = 14;

constexpr uint8_t aux = 27;
constexpr uint8_t LoRA_RX = 26;
constexpr uint8_t LoRA_TX = 25;
constexpr uint8_t m0 = 32;
constexpr uint8_t m1 = 33;

constexpr uint8_t settingCmd[] = {
    0xC0, 0x00, 0x08,
    0x00, 0x00,
    0xEC,        // UART 115200bps + SF8/BW125
    0x81,        // 64byte sub-packet + 13dBm
    0x04,        // BW125時 CH4 = 921.4MHz
    0xC3,        // 元設定を維持
    0x00, 0x00}; // 本部側
constexpr uint8_t readCmd[] = {0xC1, 0x00, 0x08};

constexpr uint8_t ADD_H = 0x00;
constexpr uint8_t ADD_L = 0x00;
constexpr uint8_t CHNNL = 0x04;
constexpr uint8_t HEADER1 = 0xAA;
constexpr bool LORA_APPEND_RSSI = true;
constexpr uint8_t TX_FRAME_SIZE = 39;
constexpr uint8_t RX_FRAME_SIZE = TX_FRAME_SIZE - 3;
constexpr uint8_t CHECKSUM_START_OFFSET = 3;
constexpr uint8_t CHECKSUM_END_OFFSET = 37;
constexpr uint8_t CHECKSUM_OFFSET = 38;
constexpr uint8_t CMD_PREFIX_0 = 0x00;
constexpr uint8_t CMD_CHNNL = 0x04;

struct TelemetryData
{
  uint8_t add_h;
  uint8_t add_l;
  uint8_t chnnl;
  uint8_t header1;

  uint8_t status;
  int32_t latitude;
  int32_t longitude;
  int16_t gnss_height; // 10 m単位
  int16_t angle_speed[3];
  int16_t acceleration[3];
  int16_t integrated_angle[3];
  uint8_t air_pressure[3];
  uint8_t air_speed;
  int8_t fin_angle;
  uint8_t rssi;
};