#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "flight_kinematics.h"

// SD pins: the card is on the default VSPI bus (SCK 18, MISO 19, MOSI 23)
// with CS = SD_CS_PIN in main.cpp. SD.begin() is done once at boot by the
// mission loader; this logger only opens files on the already-mounted card.

namespace raven {

class TelemetryLogger {
  public:
    // Creates the next free /logNNN.csv (one file per boot) and writes the
    // header row. The SD bus is shared, so every access takes sdMutex.
    bool begin(SemaphoreHandle_t sdMutex);

    // Appends one CSV row. The file stays open; data is flushed to the card
    // once a second, so a power cut loses at most ~1 s of log.
    void logState(const VehicleState& st, const ActuatorCmd& cmd);

    const char *path() const { return filepath_; }

  private:
    char filepath_[16] = {0};
    bool initialized_ = false;
    SemaphoreHandle_t sdMutex_ = nullptr;
    File file_;
    uint32_t lastFlushMs_ = 0;
};

} // namespace raven
