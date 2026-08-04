#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"

void start_decode_task(TaskHandle_t notify_task);
bool copy_latest_telemetry(TelemetryData &destination);
uint32_t get_telemetry_sequence();
