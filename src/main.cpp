#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

constexpr uint8_t LoRA_RX = 18;
constexpr uint8_t LoRA_TX = 19;
constexpr uint8_t AUX = 23;
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr uint8_t PAYLOAD_SIZE = 25;

SemaphoreHandle_t TlmMutex;

struct TelemetryData
{
  uint8_t status;
  int32_t latitude;
  int32_t longitude;
  int16_t angle_speed[3];
  int16_t acceleration[3];
  uint8_t air_pressure[3];
  uint8_t rssi;
} TlmData;

TelemetryData TLM;
TelemetryData print_TLM;

void decode_task(void *pvParameters);
TelemetryData buffer_to_telemetry(uint8_t buffer[PAYLOAD_SIZE]);
void print_telemetry_task(void *pvParameters);

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, LoRA_RX, LoRA_TX);
  TlmMutex = xSemaphoreCreateMutex();
  xTaskCreateUniversal(decode_task, "decode_task", 4096, NULL, 3, NULL, 0);
  xTaskCreateUniversal(print_telemetry_task, "print_telemetry_task", 4096, NULL, 1, NULL, 0);
}

void loop()
{
  if (Serial.available())
  {
    char cmd = Serial.read();
    Serial1.write(0x00);
    Serial1.write(0x00);
    Serial1.write(0x03);
    Serial1.write(cmd);
    Serial.println(cmd);
  }
  delay(300);
}

void decode_task(void *pvParameters)
{
  uint8_t checksum = 0;
  uint8_t state = 0;
  uint8_t buffer[PAYLOAD_SIZE];
  int bufferIndex = 0;
  while (1)
  {
    while (Serial1.available() > 0)
    {
      uint8_t c = Serial1.read();

      switch (state)
      {
      case 0: // ヘッダー1待ち
        if (c == HEADER1)
          state = 1;
        break;

      case 1: // ヘッダー2待ち
        if (c == HEADER2)
        {
          state = 2; // ヘッダーが揃ったら次へ
          bufferIndex = 0;
          checksum = HEADER1 ^ HEADER2; // チェックサム初期化
        }
        else
        {
          Serial.println("invalid header");
          state = 0; // ノイズだった場合は最初からやり直し
        }
        break;

      case 2: // ペイロードの受信
        buffer[bufferIndex++] = c;
        checksum ^= c;

        if (bufferIndex >= PAYLOAD_SIZE - 1)
        {
          // Serial.println("to checksum");
          state = 3; // データが規定サイズに達したらチェックサム確認へ
        }
        break;

      case 3: // チェックサムの確認
        if (c == checksum)
        {
          state = 4;
        }
        else
        {
          // Serial.println("Checksum Error");
          // for (int i = 0; i < PAYLOAD_SIZE; i++)
          // {
          //   Serial.print(buffer[i]);
          // }
          state = 0; // データが破損しているので破棄
        }
        break;

      case 4: // RSSIの受信
        // dBm換算は(int)値 - 256
        buffer[bufferIndex++] = c;
        int rssi_dBm = (int)c - 256;
        TelemetryData tlm = buffer_to_telemetry(buffer);
        xSemaphoreTake(TlmMutex, portMAX_DELAY);
        TLM = tlm;
        xSemaphoreGive(TlmMutex);
        // Serial.println(latitude);
        // Serial.println(longitude);
        // データを表示
        // Serial.printf("RSSI: %d dBm", rssi_dBm);
        state = 0; // 次のパケット待ちに戻る
        bufferIndex = 0;
        break;
      }
    }
    delay(300);
  }
}

TelemetryData buffer_to_telemetry(uint8_t buffer[PAYLOAD_SIZE])
{
  int idx = 0; // バッファの読み取り位置
  int32_t lat_i32;
  int32_t lon_i32;
  int16_t angle_speed[3];
  int16_t acceleration[3];
  uint8_t air_pressure_24[3];
  uint8_t rssi;
  TelemetryData tlm;

  uint8_t status = buffer[idx];
  idx += 1;

  memcpy(&lat_i32, &buffer[idx], sizeof(uint32_t));
  idx += 4;

  memcpy(&lon_i32, &buffer[idx], sizeof(uint32_t));
  idx += 4;

  memcpy(angle_speed, &buffer[idx], sizeof(int16_t) * 3);
  idx += 6;

  memcpy(acceleration, &buffer[idx], sizeof(int16_t) * 3);
  idx += 6;

  memcpy(air_pressure_24, &buffer[idx], sizeof(uint8_t) * 3);
  idx += 3;

  rssi = buffer[idx];
  idx += 1;

  tlm.status = status;
  tlm.latitude = lat_i32;
  tlm.longitude = lon_i32;
  memcpy(&tlm.angle_speed, &angle_speed, sizeof(angle_speed));
  memcpy(&tlm.acceleration, &acceleration, sizeof(acceleration));
  memcpy(&tlm.air_pressure, &air_pressure_24, sizeof(air_pressure_24));
  tlm.rssi = rssi;
  return tlm;
}

void print_telemetry_task(void *pvParameters)
{
  while (1)
  {
    xSemaphoreTake(TlmMutex, portMAX_DELAY);
    print_TLM = TLM;
    xSemaphoreGive(TlmMutex);
    uint8_t status = print_TLM.status;
    uint8_t camera_log_st = status & 1;          // 0でon 1でoff
    uint8_t camera_power_st = (status >> 1) & 1; // 0でon 1でoff
    uint8_t raspi_power_st = (status >> 2) & 1;  // 0でon 1でoff
    uint8_t can_para_log_st = (status >> 3) & 1; // 1なら正常 0なら異常
    uint8_t can_camera_st = (status >> 4) & 1;   // 1なら正常 0なら異常
    uint8_t sequence_st = (status >> 5) & 1;     // 1ならスタート 0なら待機
    uint8_t liftoff_st = (status >> 6) & 1;      // 1なら離床
    uint8_t parachute_st = (status >> 7) & 1;    // 1なら開傘

    // ステータスの出力
    Serial.printf("Status: CamLog:%s CamPwr:%s PiPwr:%s ParaLog:%s CanCam:%s Sequence:%s Liftoff:%s Para:%s\r\n",
                  camera_log_st == 0 ? "ON" : "OFF",
                  camera_power_st == 0 ? "ON" : "OFF",
                  raspi_power_st == 0 ? "ON" : "OFF",
                  can_para_log_st == 1 ? "OK" : "\e[31mCAN_P_ERR\e[m",
                  can_camera_st == 1 ? "OK" : "\e[31mCAN_C_ERR\e[m",
                  sequence_st == 1 ? "\e[32mSTART\e[m" : "WAIT",
                  liftoff_st == 1 ? "\e[31mDETECT\e[m" : "WAIT",
                  parachute_st == 1 ? "\e[31mOPEN\e[m" : "CLOSE");

    // 緯度・経度の出力
    Serial.printf("Lat: %ld, Lon: %ld\r\n", print_TLM.latitude, print_TLM.longitude);

    //  角速度の出力
    Serial.printf("Angle Speed  [X: %d, Y: %d, Z: %d]\r\n",
                  print_TLM.angle_speed[0], print_TLM.angle_speed[1], print_TLM.angle_speed[2]);

    //  加速度の出力
    Serial.printf("Acceleration [X: %d, Y: %d, Z: %d]\r\n",
                  print_TLM.acceleration[0], print_TLM.acceleration[1], print_TLM.acceleration[2]);

    //  気圧の出力(raw dataと変換後)
    Serial.printf("Air Pressure [0: %d, 1: %d, 2: %d]\r\n",
                  print_TLM.air_pressure[0], print_TLM.air_pressure[1], print_TLM.air_pressure[2]);
    Serial.printf("Air Pressure: %d Pa\r\n",
                  print_TLM.air_pressure[0] + 256 * print_TLM.air_pressure[1] + 65536 * print_TLM.air_pressure[2]);

    //  RSSIの出力 (dBm)
    Serial.printf("RSSI: %d dBm\r\n", (int)print_TLM.rssi - 256);

    delay(300);
  }
}