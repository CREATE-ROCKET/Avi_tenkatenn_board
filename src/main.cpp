#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "config.h"
#include "decode.h"

TaskHandle_t printTaskHandle = NULL;
TelemetryData print_TLM;
QueueHandle_t commandQueue = nullptr;

namespace
{
  constexpr UBaseType_t COMMAND_QUEUE_LENGTH = 4;
}

void print_telemetry_task(void *pvParameters);
void command_send_task(void *pvParameters);
bool wait_aux_high(uint32_t timeout_ms);
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
  commandQueue = xQueueCreate(COMMAND_QUEUE_LENGTH, sizeof(uint8_t));
  if (commandQueue == nullptr)
  {
    Serial.println("failed to create command queue");
    return;
  }

  xTaskCreateUniversal(
      command_send_task,
      "command_send_task",
      3072,
      NULL,
      2,
      nullptr,
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

  while (Serial.available() > 0)
  {
    const char cmd = static_cast<char>(Serial.read());

    if (cmd == '\r' || cmd == '\n')
    {
      continue;
    }

    const uint8_t command = static_cast<uint8_t>(cmd);
    if (commandQueue == nullptr ||
        xQueueSend(commandQueue, &command, 0) != pdPASS)
    {
      Serial.println("command queue full");
      continue;
    }

    Serial.print("command queued: ");
    Serial.println(cmd);
  }

  delay(10);
}

bool wait_aux_high(uint32_t timeout_ms)
{
  const uint32_t started_at = millis();

  while (digitalRead(aux) != HIGH)
  {
    if (millis() - started_at >= timeout_ms)
    {
      return false;
    }

    vTaskDelay(pdMS_TO_TICKS(AUX_POLL_INTERVAL_MS));
  }

  return true;
}

void command_send_task(void *pvParameters)
{
  uint8_t command = 0;

  while (true)
  {
    if (xQueueReceive(commandQueue, &command, portMAX_DELAY) != pdPASS)
    {
      continue;
    }

    const uint32_t sequence_at_dequeue = get_telemetry_sequence();
    while (get_telemetry_sequence() <= sequence_at_dequeue)
    {
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    Serial.println("telemetry received, waiting AUX");

    if (!wait_aux_high(AUX_TIMEOUT_MS))
    {
      Serial.print("AUX timeout, command cancelled: ");
      Serial.println(static_cast<char>(command));
      continue;
    }

    const uint8_t checksum =
        static_cast<uint8_t>(HEADER_UP ^ command);

    const uint8_t send_data[] = {
        CMD_PREFIX_0,
        CMD_PREFIX_0,
        CMD_CHNNL,
        HEADER_UP,
        command,
        checksum};

    Serial1.write(send_data, sizeof(send_data));
    Serial1.flush();

    if (!wait_aux_high(AUX_TIMEOUT_MS))
    {
      Serial.print("AUX timeout after send: ");
      Serial.println(static_cast<char>(command));
      continue;
    }

    Serial.print("send cmd: ");
    Serial.println(static_cast<char>(command));
  }
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
  uint32_t receive_interval_ms = 0;
  bool has_receive_interval = false;

  while (1)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if (!copy_latest_telemetry(
            print_TLM,
            receive_interval_ms,
            has_receive_interval))
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

    if (has_receive_interval)
    {
      Serial.print("Receive interval: ");
      Serial.print(receive_interval_ms / 1000.0, 3);
      Serial.println(" s");
    }
    else
    {
      Serial.println("Receive interval: N/A");
    }

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
