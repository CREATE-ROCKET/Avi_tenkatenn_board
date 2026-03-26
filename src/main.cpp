#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

constexpr uint8_t LoRA_RX = 27;
constexpr uint8_t LoRA_TX = 26;
// constexpr uint8_t AUX = 14;
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr uint8_t PAYLOAD_SIZE = 25;

constexpr uint8_t CMD_PREFIX_0 = 0x00;
constexpr uint8_t CMD_ERASE_PREFIX = 0x03;

SemaphoreHandle_t TlmMutex;

// コマンド用のステートマシン状態定義
enum EraseState
{
  STATE_IDLE,        // 通常のコマンド待ち
  STATE_WAIT_CONFIRM // 'x'が押され、'y'か'n'の確認待ち
};
EraseState current_loop_state = STATE_IDLE;

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

TelemetryData TLM = {7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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

    switch (current_loop_state)
    {
    case STATE_IDLE:
      if (cmd == 'x')
      {
        Serial.println("Do you want to erase flash? (y/n)");
        // 次のループからは確認待ち状態になる
        current_loop_state = STATE_WAIT_CONFIRM;
      }
      else
      {
        // 'x' 以外の通常のコマンド送信
        Serial1.write(CMD_PREFIX_0);     // add_u
        Serial1.write(CMD_PREFIX_0);     // add_l
        Serial1.write(CMD_ERASE_PREFIX); // channel
        Serial1.write(cmd);
        Serial.println(cmd);
      }
      break;

    case STATE_WAIT_CONFIRM:
      if (cmd == 'y')
      {
        Serial.println("erase start");
        Serial1.write(CMD_PREFIX_0);
        Serial1.write(CMD_PREFIX_0);
        Serial1.write(CMD_ERASE_PREFIX);
        Serial1.write('x'); // 消去コマンド本体
        Serial.println("x");
        current_loop_state = STATE_IDLE; // 処理が終わったら通常状態に戻る
      }
      else if (cmd == 'n')
      {
        Serial.println("erase denied");
        current_loop_state = STATE_IDLE; // キャンセルして通常状態に戻る
      }
      else
      {
        // 'y' でも 'n' でもない無効な入力の場合
        Serial.println("try again\n press x  to erase flash");
        current_loop_state = STATE_IDLE; // 一旦リセットする
      }
      break;
    }
  }

  delay(50);
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
    delay(50);
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

  memcpy(&lat_i32, &buffer[idx], sizeof(int32_t));
  idx += 4;

  memcpy(&lon_i32, &buffer[idx], sizeof(int32_t));
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
  memcpy(tlm.angle_speed, angle_speed, sizeof(angle_speed));
  memcpy(tlm.acceleration, acceleration, sizeof(acceleration));
  memcpy(tlm.air_pressure, air_pressure_24, sizeof(air_pressure_24));
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
    Serial.printf("Status\r\nCamLog:%s CamPwr:%s PiPwr:%s CanPara:%s CanCam:%s Sequence:%s Liftoff:%s Para:%s\r\n",
                  camera_log_st == 0 ? "ON" : "OFF",
                  camera_power_st == 0 ? "ON" : "OFF",
                  raspi_power_st == 0 ? "ON" : "OFF",
                  can_para_log_st == 1 ? "OK" : "\033[31mCAN_P_ERR\033[m",
                  can_camera_st == 1 ? "OK" : "\033[31mCAN_C_ERR\033[m",
                  sequence_st == 1 ? "\033[32mSTART\033[m" : "WAIT",
                  liftoff_st == 1 ? "\033[31mDETECT\033[m" : "WAIT",
                  parachute_st == 1 ? "\033[31mOPEN\033[m" : "CLOSE");

    // 緯度・経度の出力
    Serial.printf("Latitude: %ld, Longitude: %ld\r\n", print_TLM.latitude, print_TLM.longitude);

    //  角速度の出力
    Serial.printf("Angular Velocity  [X: %d, Y: %d, Z: %d]\r\n",
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
    Serial.println();

    delay(1500);
  }
}