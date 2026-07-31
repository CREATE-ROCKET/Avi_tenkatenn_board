#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "decode.h"

TaskHandle_t printTaskHandle = NULL;
TelemetryData print_TLM;

void print_telemetry_task(void *pvParameters);
void setup_lora_settings();

void setup()
{
  Serial.begin(115200);
  pinMode(aux, INPUT);
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);

  pinMode(top_led, OUTPUT);
  pinMode(liftoff_led, OUTPUT);
  pinMode(control_led, OUTPUT);
  pinMode(update_led, OUTPUT);

  if (BOOT_MODE == BootMode::LoRaSetup)
  {
    Serial1.begin(9600, SERIAL_8N1, LoRA_RX, LoRA_TX);
    setup_lora_settings();
    return;
  }

  Serial1.begin(115200, SERIAL_8N1, LoRA_RX, LoRA_TX);
  digitalWrite(m0, LOW);
  digitalWrite(m1, LOW);

  digitalWrite(top_led, LOW);
  digitalWrite(liftoff_led, LOW);
  digitalWrite(control_led, LOW);
  digitalWrite(update_led, LOW);

  xTaskCreateUniversal(
      print_telemetry_task,
      "print_telemetry_task",
      4096,
      NULL,
      1,
      &printTaskHandle,
      0);

  start_decode_task(printTaskHandle);
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
    uint8_t b = Serial1.read();

    if (b < 0x10)
    {
      Serial.print("0");
    }
    Serial.print(b, HEX);
    Serial.print(" ");
  }
  Serial.println("LoRa setup finished. Please set BOOT_MODE to Communication and upload again.");
}

void print_telemetry_task(void *pvParameters)
{
  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!copy_latest_telemetry(print_TLM))
    {
      continue;
    }

    uint8_t status = print_TLM.status;

    uint8_t top_detect = status & 1;
    uint8_t main_power_st = (status >> 1) & 1;
    uint8_t emergency_power_st = (status >> 2) & 1;
    uint8_t control_st = (status >> 3) & 1;
    uint8_t can_st = (status >> 4) & 1;
    uint8_t sequence_st = (status >> 5) & 1;
    uint8_t liftoff_st = (status >> 6) & 1;
    uint8_t parachute_st = (status >> 7) & 1;

    Serial.printf("Top : %s\r\n", top_detect ? "Detected" : "Yet");
    Serial.printf("Main power: %s\r\n", main_power_st ? "ON" : "OFF");
    Serial.printf("Emergency power: %s\r\n", emergency_power_st ? "ON" : "OFF");
    Serial.printf("Control: %s\r\n", control_st ? "Enabled" : "Disabled");
    Serial.printf("CAN: %s\r\n", can_st ? "OK" : "ERROR");
    Serial.printf("Sequence: %s\r\n", sequence_st ? "Running" : "IDLE");
    Serial.printf("Liftoff: %s\r\n", liftoff_st ? "Detected" : "Yet");
    Serial.printf("Parachute: %s\r\n", parachute_st ? "OPEN" : "CLOSE");

    if (top_detect)
      digitalWrite(top_led, HIGH);
    if (liftoff_st)
      digitalWrite(liftoff_led, HIGH);
    if (control_st)
      digitalWrite(control_led, HIGH);
    double lat_deg = print_TLM.latitude / 10000000.0;
    double lon_deg = print_TLM.longitude / 10000000.0;

    int32_t height_m = (int32_t)print_TLM.gnss_height * 10;

    Serial.printf("Latitude: %.7f deg\r\n", lat_deg);
    Serial.printf("Longitude: %.7f deg\r\n", lon_deg);
    Serial.printf("GNSS height: %ld m\r\n", (long)height_m);
    Serial.printf(
        "Angle speed: [%d, %d, %d]\r\n",
        (int)print_TLM.angle_speed[0],
        (int)print_TLM.angle_speed[1],
        (int)print_TLM.angle_speed[2]);
    Serial.printf(
        "Acceleration: [%d, %d, %d]\r\n",
        (int)print_TLM.acceleration[0],
        (int)print_TLM.acceleration[1],
        (int)print_TLM.acceleration[2]);
    Serial.printf(
        "Integrated angle: [%d, %d, %d]\r\n",
        (int)print_TLM.integrated_angle[0],
        (int)print_TLM.integrated_angle[1],
        (int)print_TLM.integrated_angle[2]);
    Serial.printf(
        "Air pressure: [%u, %u, %u]\r\n",
        (unsigned int)print_TLM.air_pressure[0],
        (unsigned int)print_TLM.air_pressure[1],
        (unsigned int)print_TLM.air_pressure[2]);
    Serial.printf("Air speed: %u\r\n", (unsigned int)print_TLM.air_speed);
    Serial.printf("Fin angle: %d\r\n", (int)print_TLM.fin_angle);

    if (LORA_APPEND_RSSI)
    {
      Serial.printf(
          "RSSI: %d dBm\r\n",
          (int)print_TLM.rssi - 256);
    }

    Serial.println();
  }
}
