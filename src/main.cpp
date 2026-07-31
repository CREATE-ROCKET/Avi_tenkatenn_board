#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>

#include "config.h"

SemaphoreHandle_t TlmMutex;
TaskHandle_t printTaskHandle = NULL;
TelemetryData TLM = {
    ADD_H,
    ADD_L,
    CHNNL,
    HEADER1,
    0,
    0,
    0,
    0,
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    0,
    0,
    0};

TelemetryData print_TLM;

void decode_task(void *pvParameters);
void print_telemetry_task(void *pvParameters);

uint8_t calc_checksum_from_full_frame(const uint8_t *full_frame);
bool verify_checksum(const uint8_t *full_frame);
void publish_telemetry(const uint8_t *full_frame, uint8_t rssi);
TelemetryData buffer_to_telemetry(const uint8_t *full_frame, uint8_t rssi);
void setup_lora_settings();

void setup()
{
  Serial.begin(115200);
  pinMode(aux, INPUT);
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);

  if (BOOT_MODE == BootMode::LoRaSetup)
  {
    Serial1.begin(9600, SERIAL_8N1, LoRA_RX, LoRA_TX);
    setup_lora_settings();
    return;
  }

  Serial1.begin(115200, SERIAL_8N1, LoRA_RX, LoRA_TX);
  digitalWrite(m0, LOW);
  digitalWrite(m1, LOW);

  TlmMutex = xSemaphoreCreateMutex();

  xTaskCreateUniversal(
      decode_task,
      "decode_task",
      4096,
      NULL,
      3,
      NULL,
      0);

  xTaskCreateUniversal(
      print_telemetry_task,
      "print_telemetry_task",
      4096,
      NULL,
      1,
      &printTaskHandle,
      0);
  delay(100);
}

void loop()
{
  if (BOOT_MODE == BootMode::LoRaSetup)
  {
    delay(1000);
    return;
  }

  if (Serial.available())
  {
    char cmd = Serial.read();

    // シリアルモニタの改行を無視
    if (cmd == '\r' || cmd == '\n')
    {
      delay(50);
      return;
    }

    // E220固定送信用コマンド
    Serial1.write(CMD_PREFIX_0);
    Serial1.write(CMD_PREFIX_0);
    Serial1.write(CMD_CHNNL);

    Serial1.write((uint8_t)cmd);

    Serial.print("send cmd: ");
    Serial.println(cmd);
  }
  delay(50);
}

void setup_lora_settings()
{
  digitalWrite(m0, HIGH);
  digitalWrite(m1, HIGH);
  delay(100);

  while (Serial1.available() > 0)
  {
    Serial1.read();
  }

  Serial.println("Writing LoRa settings...");
  Serial1.write(settingCmd, sizeof(settingCmd));
  Serial1.flush();
  delay(200);
  while (Serial1.available() > 0)
  {
    byte b = Serial1.read();

    if (b < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(b, HEX);
    Serial.print(" ");
  }
  Serial.println("LoRa setup finished. Please set BOOT_MODE to Communication and upload again.");
}

uint8_t calc_checksum_from_full_frame(const uint8_t *full_frame)
{
  uint8_t checksum = 0;

  for (uint8_t i = CHECKSUM_START_OFFSET; i <= CHECKSUM_END_OFFSET; i++)
  {
    checksum ^= full_frame[i];
  }

  return checksum;
}

bool verify_checksum(const uint8_t *full_frame)
{
  uint8_t calculated = calc_checksum_from_full_frame(full_frame);
  uint8_t received = full_frame[CHECKSUM_OFFSET];

  return calculated == received;
}

void decode_task(void *pvParameters)
{
  uint8_t state = 0;
  uint8_t rx_index = 0;

  // offset 0..38 の完全なフレームに復元して扱う
  uint8_t full_frame[TX_FRAME_SIZE];

  while (1)
  {
    while (Serial1.available() > 0)
    {
      uint8_t c = Serial1.read();
      switch (state)
      {
      case 0:
        if (c == HEADER1)
        {
          full_frame[0] = ADD_H;
          full_frame[1] = ADD_L;
          full_frame[2] = CHNNL;
          full_frame[3] = c;

          rx_index = 4;
          state = 1;
        }
        break;

      case 1:
        full_frame[rx_index++] = c;

        if (rx_index >= TX_FRAME_SIZE)
        {
          if (verify_checksum(full_frame))
          {
            if (LORA_APPEND_RSSI)
            {
              state = 2;
            }
            else
            {
              publish_telemetry(full_frame, 0);
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
      {
        uint8_t rssi = c;
        publish_telemetry(full_frame, rssi);

        state = 0;
        rx_index = 0;
        break;
      }

      default:
        state = 0;
        rx_index = 0;
        break;
      }
    }
    delay(15);
  }
}

void publish_telemetry(const uint8_t *full_frame, uint8_t rssi)
{
  TelemetryData tlm = buffer_to_telemetry(full_frame, rssi);

  xSemaphoreTake(TlmMutex, portMAX_DELAY);
  TLM = tlm;
  xSemaphoreGive(TlmMutex);

  if (printTaskHandle != NULL)
  {
    xTaskNotifyGive(printTaskHandle);
  }
}

TelemetryData buffer_to_telemetry(const uint8_t *buffer, uint8_t rssi)
{
  TelemetryData tlm;

  tlm.add_h = buffer[0];
  tlm.add_l = buffer[1];
  tlm.chnnl = buffer[2];
  tlm.header1 = buffer[3];

  tlm.status = buffer[4];

  memcpy(&tlm.latitude, &buffer[5], sizeof(int32_t));
  memcpy(&tlm.longitude, &buffer[9], sizeof(int32_t));
  memcpy(&tlm.gnss_height, &buffer[13], sizeof(int16_t));

  memcpy(tlm.angle_speed, &buffer[15], sizeof(int16_t) * 3);
  memcpy(tlm.acceleration, &buffer[21], sizeof(int16_t) * 3);
  memcpy(tlm.integrated_angle, &buffer[27], sizeof(int16_t) * 3);

  memcpy(tlm.air_pressure, &buffer[33], sizeof(uint8_t) * 3);

  tlm.air_speed = buffer[36];

  memcpy(&tlm.fin_angle, &buffer[37], sizeof(int8_t));

  tlm.rssi = rssi;

  return tlm;
}

void print_telemetry_task(void *pvParameters)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    xSemaphoreTake(TlmMutex, portMAX_DELAY);
    print_TLM = TLM;
    xSemaphoreGive(TlmMutex);

    uint8_t status = print_TLM.status;

    uint8_t camera_log_st = status & 1;
    uint8_t camera_power_st = (status >> 1) & 1;
    uint8_t raspi_power_st = (status >> 2) & 1;
    uint8_t can_para_log_st = (status >> 3) & 1;
    uint8_t can_camera_st = (status >> 4) & 1;
    uint8_t sequence_st = (status >> 5) & 1;
    uint8_t liftoff_st = (status >> 6) & 1;
    uint8_t parachute_st = (status >> 7) & 1;

    double lat_deg = print_TLM.latitude / 10000000.0;
    double lon_deg = print_TLM.longitude / 10000000.0;

    int32_t height_m = (int32_t)print_TLM.gnss_height * 10;

    Serial.printf(
        "GNSS [Lat: %.5f deg, Lon: %.5f deg, Height: %ld m]\r\n",
        lat_deg,
        lon_deg,
        (long)height_m);

    if (LORA_APPEND_RSSI)
    {
      Serial.printf(
          "RSSI: %d dBm\r\n",
          (int)print_TLM.rssi - 256);
    }

    Serial.println();
  }
}
