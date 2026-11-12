#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
extern SemaphoreHandle_t sensorBusMutex;
struct SensorBusLock {
  SensorBusLock() { xSemaphoreTake(sensorBusMutex, portMAX_DELAY); }
  ~SensorBusLock() { xSemaphoreGive(sensorBusMutex); }
};
