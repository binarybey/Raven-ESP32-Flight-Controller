#pragma once

#include <Arduino.h>
#include <SD.h>
#include <vector>
#include "flight_kinematics.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace raven {

enum class FCodeCmdType : uint8_t {
    WAYPOINT,
    LAND
};

struct MissionCmd {
    FCodeCmdType type;
    float lat;
    float lon;
    float alt;
};

class FCodeInterpreter {
  public:
    // Reads the entire file into memory during setup, then closes the file.
    bool begin(uint8_t csPin, const char* filepath, SemaphoreHandle_t spMutex, SemaphoreHandle_t sdMutex, Setpoints* globalSp);
    
    // Evaluates memory-stored targets against the live vehicle state.
    void update(const VehicleState& st);
    
    bool isComplete() const { return current_step_ >= mission_.size(); }

  private:
    bool parseLine(const String& line, MissionCmd& cmdOut);
    bool checkTargetMet(const VehicleState& st, const MissionCmd& target) const;

    std::vector<MissionCmd> mission_;
    size_t current_step_ = 0;
    
    SemaphoreHandle_t spMutex_;
    Setpoints* globalSp_;
    float acceptance_radius_ = 2.5f; // meters
};

} // namespace raven