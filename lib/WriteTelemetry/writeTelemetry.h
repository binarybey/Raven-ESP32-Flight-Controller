#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "flight_kinematics.h"

// --- SD Card SPI Pin Mapping ---
constexpr uint8_t SD_CS   = 5;
constexpr uint8_t SD_MOSI = 23;
constexpr uint8_t SD_MISO = 19;
constexpr uint8_t SD_SCK  = 18;

namespace raven {

class TelemetryLogger {
  public:
    // Opens or creates the CSV log file and writes the header row
    bool begin(const char* filepath, SemaphoreHandle_t sdMutex);

    // Appends a single row of telemetry to the SD card
    void logState(const VehicleState& st, const ActuatorCmd& cmd);

  private:
    char filepath_[32];
    bool initialized_ = false;
    SemaphoreHandle_t sdMutex_;
};

} // namespace raven