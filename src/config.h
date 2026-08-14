#pragma once

#include <stdint.h>

enum class BootMode
{
  Communication,
  LoRaSetup,
  LoRaReadback
};

constexpr BootMode BOOT_MODE = BootMode::Communication;

constexpr uint8_t top_led = 19;
constexpr uint8_t liftoff_led = 21;
constexpr uint8_t control_led = 13;
constexpr uint8_t update_led = 14;
constexpr uint32_t TELEMETRY_TIMEOUT_MS = 5000;
constexpr uint32_t AUX_TIMEOUT_MS = 2000;
constexpr uint32_t AUX_POLL_INTERVAL_MS = 1;
// 実測した周期packet完了後の成功域に収める。
constexpr uint32_t UPLINK_BOUNDARY_FRESH_US = 20000;
// B1停止後に3件のA0〜A4を待てる有限時間とし、通常commandを残留させない。
constexpr uint32_t UPLINK_BOUNDARY_TIMEOUT_MS = 2200;
// B1を1件取りこぼしてもPeriodic modeへ誤遷移しない連続受信数。
constexpr uint8_t UPLINK_PERIODIC_ACTIVATION_COUNT = 3;

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
constexpr bool LORA_APPEND_RSSI = true;
