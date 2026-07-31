#include "decode.h"

#include <Arduino.h>
#include <freertos/semphr.h>
#include <string.h>

namespace
{
SemaphoreHandle_t telemetry_mutex = nullptr;
TaskHandle_t notify_task_handle = nullptr;
TelemetryData telemetry = {};

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
  xSemaphoreGive(telemetry_mutex);

  if (notify_task_handle != nullptr)
  {
    xTaskNotifyGive(notify_task_handle);
  }
}

void decode_task(void *pvParameters)
{
  uint8_t state = 0;
  uint8_t rx_index = 0;
  uint8_t frame[TX_FRAME_SIZE];

  while (true)
  {
    while (Serial1.available() > 0)
    {
      const uint8_t value = Serial1.read();
      switch (state)
      {
      case 0:
        if (value == HEADER1)
        {
          frame[0] = ADD_H;
          frame[1] = ADD_L;
          frame[2] = CHNNL;
          frame[3] = value;
          rx_index = 4;
          state = 1;
        }
        break;
      case 1:
        frame[rx_index++] = value;
        if (rx_index >= TX_FRAME_SIZE)
        {
          if (calc_checksum(frame) == frame[CHECKSUM_OFFSET])
          {
            if (LORA_APPEND_RSSI)
            {
              state = 2;
            }
            else
            {
              publish_telemetry(frame, 0);
              state = 0;
              rx_index = 0;
            }
          }
          else
          {
            state = 0;
            rx_index = 0;
          }
        }
        break;
      case 2:
        publish_telemetry(frame, value);
        state = 0;
        rx_index = 0;
        break;
      default:
        state = 0;
        rx_index = 0;
        break;
      }
    }
    delay(15);
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
