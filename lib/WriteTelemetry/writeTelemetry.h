#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "flight_kinematics.h"

// SD pins: the card is on the default VSPI bus (SCK 18, MISO 19, MOSI 23)
// with CS = SD_CS_PIN in main.cpp. SD.begin() is done once in setup(); this
// logger only opens files on the already-mounted card.

// What the telemetry line, the SD log and the 'status' command show -
// TaskFlightControl fills it, TaskTelemetry and TaskConsole read it (under
// telemMutex).
struct TelemetrySnapshot {
    raven::VehicleState st;
    raven::ActuatorCmd  out;
    raven::RawSensors   raw;         // sensor health flags, GNSS fix/sats/HDOP
    uint32_t            terrain_ms;  // last DEM lookup time
    float               loop_hz;     // measured control rate
};

// The 10 Hz status line on Serial (~210 bytes = 2 kB/s, well inside 115200
// baud). Also the natural place for an RF telemetry downlink.
void printTelemetry(const TelemetrySnapshot &t);

// Boot report: mass, hover thrust share, wing stall, config warnings.
void printVehicleReport(const raven::FlightKinematics &fk, const raven::VehicleConfig &cfg,
                        uint32_t configIssues);

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
