#include "decode.h"

#include <Arduino.h>
#include <freertos/semphr.h>
#include <string.h>

namespace
{
  SemaphoreHandle_t telemetry_mutex = nullptr;
  TaskHandle_t notify_task_handle = nullptr;
  TelemetryData telemetry = {};
  uint32_t telemetry_sequence = 0;
  uint32_t last_telemetry_at = 0;
  bool telemetry_is_active = false;

  uint8_t calc_checksum(const uint8_t *frame)
  {
    uint8_t checksum = 0;
    for (uint8_t i = CHECKSUM_START_OFFSET; i <= CHECKSUM_END_OFFSET; i++)
    {
      checksum ^= frame[i];
    }
    return checksum;
  }

  TelemetryData decode_telemetry(const uint8_t *buffer, uint8_t rssi)
  {
    TelemetryData result;
    result.add_h = buffer[0];
    result.add_l = buffer[1];
    result.chnnl = buffer[2];
    result.header1 = buffer[3];
    result.status = buffer[4];
    memcpy(&result.latitude, &buffer[5], sizeof(result.latitude));
    memcpy(&result.longitude, &buffer[9], sizeof(result.longitude));
    memcpy(&result.gnss_height, &buffer[13], sizeof(result.gnss_height));
    memcpy(result.angle_speed, &buffer[15], sizeof(result.angle_speed));
    memcpy(result.acceleration, &buffer[21], sizeof(result.acceleration));
    memcpy(result.integrated_angle, &buffer[27], sizeof(result.integrated_angle));
    memcpy(result.air_pressure, &buffer[33], sizeof(result.air_pressure));
    result.air_speed = buffer[36];
    memcpy(&result.fin_angle, &buffer[37], sizeof(result.fin_angle));
    result.rssi = rssi;
    return result;
  }

  void publish_telemetry(const uint8_t *frame, uint8_t rssi)
  {
    const TelemetryData decoded = decode_telemetry(frame, rssi);
    xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
    telemetry = decoded;
    telemetry_sequence++;
    xSemaphoreGive(telemetry_mutex);

    last_telemetry_at = millis();
    telemetry_is_active = true;
    digitalWrite(update_led, HIGH);

    if (notify_task_handle != nullptr)
    {
      xTaskNotifyGive(notify_task_handle);
    }
  }

  void decode_task(void *pvParameters)
  {
    constexpr uint32_t FRAME_GAP_TIMEOUT_MS = 100;

    uint8_t state = 0;
    uint8_t rx_index = 0;
    uint8_t frame[TX_FRAME_SIZE] = {};

    uint32_t last_byte_at = 0;

    while (true)
    {
      /*
       * フレーム受信途中で一定時間byteが来なければ破棄する。
       *
       * これがないと、欠落したフレームと次のフレームが結合され、
       * チェックサム位置やRSSI位置がずれる。
       */
      if (state != 0 &&
          millis() - last_byte_at >= FRAME_GAP_TIMEOUT_MS)
      {
        state = 0;
        rx_index = 0;
      }

      while (Serial1.available() > 0)
      {
        const int read_value = Serial1.read();

        if (read_value < 0)
        {
          break;
        }

        const uint8_t value =
            static_cast<uint8_t>(read_value);

        last_byte_at = millis();

        switch (state)
        {
        /*
         * ヘッダー0xAA待ち
         */
        case 0:
          if (value == HEADER1)
          {
            // E220の固定送信用3 bytesは無線受信側では届かないため、
            // 受信側で復元する。
            frame[0] = ADD_H;
            frame[1] = ADD_L;
            frame[2] = CHNNL;
            frame[3] = value;

            rx_index = 4;
            state = 1;
          }
          break;

        /*
         * Payload本体受信
         */
        case 1:
          if (rx_index >= TX_FRAME_SIZE)
          {
            state = 0;
            rx_index = 0;
            break;
          }

          frame[rx_index++] = value;

          if (rx_index == TX_FRAME_SIZE)
          {
            const uint8_t calculated =
                calc_checksum(frame);

            const uint8_t received =
                frame[CHECKSUM_OFFSET];

            if (calculated != received)
            {
              // チェックサム不一致。
              // 次の0xAAから再同期する。
              state = 0;
              rx_index = 0;
              break;
            }

            if (LORA_APPEND_RSSI)
            {
              // 次の1 byteをRSSIとして待つ。
              state = 2;
            }
            else
            {
              publish_telemetry(frame, 0);

              state = 0;
              rx_index = 0;
            }
          }
          break;

        /*
         * RSSI byte受信
         */
        case 2:
          /*
           * raw RSSI == 0は正常なRSSI値として扱わない。
           *
           * 0 - 256 = -256 dBmとなるが、これは実際の受信電力ではなく、
           * フレーム同期ずれまたはRSSI未取得を示す。
           */
          if (value != 0x00)
          {
            publish_telemetry(frame, value);
          }

          state = 0;
          rx_index = 0;
          break;

        default:
          state = 0;
          rx_index = 0;
          break;
        }
      }

      if (telemetry_is_active &&
          millis() - last_telemetry_at >= TELEMETRY_TIMEOUT_MS)
      {
        telemetry_is_active = false;
        digitalWrite(update_led, LOW);
      }

      delay(1);
    }
  }
} // namespace

void start_decode_task(TaskHandle_t notify_task)
{
  notify_task_handle = notify_task;
  telemetry_mutex = xSemaphoreCreateMutex();
  xTaskCreateUniversal(decode_task, "decode_task", 4096, nullptr, 3, nullptr, 0);
}

bool copy_latest_telemetry(TelemetryData &destination)
{
  if (telemetry_mutex == nullptr)
  {
    return false;
  }
  xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
  destination = telemetry;
  xSemaphoreGive(telemetry_mutex);
  return true;
}

uint32_t get_telemetry_sequence()
{
  if (telemetry_mutex == nullptr)
  {
    return 0;
  }

  xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
  const uint32_t sequence = telemetry_sequence;
  xSemaphoreGive(telemetry_mutex);
  return sequence;
}
