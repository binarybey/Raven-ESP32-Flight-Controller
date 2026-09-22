#include <interpretFCode.h>

namespace raven {

bool FCodeInterpreter::begin(uint8_t csPin, const char* filepath, SemaphoreHandle_t spMutex, SemaphoreHandle_t sdMutex, Setpoints* globalSp) {
    spMutex_  = spMutex;
    globalSp_ = globalSp;
    mission_.clear();

    if (xSemaphoreTake(sdMutex, portMAX_DELAY) == pdTRUE) {
        File file = SD.open(filepath, FILE_READ);
        if (!file) {
            Serial.printf("F-Code: Failed to open %s\n", filepath);
            xSemaphoreGive(sdMutex);
            return false;
        }

        // Stream the entire file into memory
        while (file.available()) {
            String line = file.readStringUntil('\n');
            line.trim(); 
            
            if (line.length() == 0 || line.startsWith("#")) continue;

            MissionCmd cmd;
            if (parseLine(line, cmd)) {
                mission_.push_back(cmd);
            } else {
                Serial.printf("F-Code Parse Error on line: %s\n", line.c_str());
            }
        }
        
        file.close();
        xSemaphoreGive(sdMutex);
    }

    Serial.printf("F-Code loaded successfully. Total commands: %u\n", mission_.size());
    return !mission_.empty();
}

void FCodeInterpreter::update(const VehicleState& st) {
    if (isComplete()) return;

    const MissionCmd& activeCmd = mission_[current_step_];

    // Push the active target into the global setpoints struct
    if (xSemaphoreTake(spMutex_, 5) == pdTRUE) {
        if (activeCmd.type == FCodeCmdType::WAYPOINT) {
            globalSp_->hold_position = false;
            // TODO: Translate lat/lon into local NED coordinates or direct guidance logic here
            // globalSp_->pitch_sp = ... 
            // globalSp_->climb_rate_sp = ...
        } else if (activeCmd.type == FCodeCmdType::LAND) {
            globalSp_->climb_rate_sp = -1.0f; // 1 m/s descent
            globalSp_->hold_position = true;
        }
        xSemaphoreGive(spMutex_);
    }

    // Check if the physical drone has reached the memory-stored target
    if (checkTargetMet(st, activeCmd)) {
        current_step_++;
        if (isComplete()) {
            Serial.println("Mission Complete.");
        } else {
            Serial.printf("Waypoint reached. Advancing to step %u\n", current_step_);
        }
    }
}

bool FCodeInterpreter::parseLine(const String& line, MissionCmd& cmdOut) {
    char buf[128];
    line.toCharArray(buf, sizeof(buf));
    
    char* token = strtok(buf, ",");
    if (!token) return false;

    if (strcmp(token, "WAYPOINT") == 0) {
        cmdOut.type = FCodeCmdType::WAYPOINT;
        char* latStr = strtok(NULL, ",");
        char* lonStr = strtok(NULL, ",");
        char* altStr = strtok(NULL, ",");
        
        if (latStr && lonStr && altStr) {
            cmdOut.lat = atof(latStr);
            cmdOut.lon = atof(lonStr);
            cmdOut.alt = atof(altStr);
            return true;
        }
    } 
    else if (strcmp(token, "LAND") == 0) {
        cmdOut.type = FCodeCmdType::LAND;
        // Land commands might not have coordinates, just trigger the descent
        cmdOut.lat = 0.0f;
        cmdOut.lon = 0.0f;
        cmdOut.alt = 0.0f; 
        return true;
    }

    return false;
}

bool FCodeInterpreter::checkTargetMet(const VehicleState& st, const MissionCmd& target) const {
    if (!st.nav_valid) return false;

    if (target.type == FCodeCmdType::LAND) {
        // Landing is complete when altitude is effectively zero and descent halts
        return (st.altitude <= 0.5f); 
    }

    bool horizontal_met = (st.distance_to_go <= acceptance_radius_);
    bool vertical_met   = (fabsf(st.altitude - target.alt) <= 1.5f);

    return (horizontal_met && vertical_met);
}

} // namespace raven