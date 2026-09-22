#include "writeTelemetry.h"

namespace raven {

bool TelemetryLogger::begin(const char* filepath, SemaphoreHandle_t sdMutex) {
    strncpy(filepath_, filepath, sizeof(filepath_) - 1);
    sdMutex_ = sdMutex;
    
    if (xSemaphoreTake(sdMutex_, portMAX_DELAY) == pdTRUE) {
        File logFile = SD.open(filepath_, FILE_APPEND);
        if (!logFile) {
            Serial.printf("Telemetry Logger: Failed to open %s\n", filepath_);
            xSemaphoreGive(sdMutex_);
            return false;
        }

        if (logFile.size() == 0) {
            logFile.println("time_ms,mode,roll,pitch,yaw,alt,climb_rate,q_dyn,alpha,thrust_L,thrust_R");
        }
        
        logFile.close();
        xSemaphoreGive(sdMutex_);
    }
    
    initialized_ = true;
    return true;
}

void TelemetryLogger::logState(const VehicleState& st, const ActuatorCmd& cmd) {
    if (!initialized_) return;

    if (xSemaphoreTake(sdMutex_, 10 / portTICK_PERIOD_MS) == pdTRUE) {
        File logFile = SD.open(filepath_, FILE_APPEND);
        if (logFile) {
            logFile.printf("%lu,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n",
                millis(),
                static_cast<int>(cmd.mode),
                st.roll * 57.2958f,
                st.pitch * 57.2958f,
                st.yaw * 57.2958f,
                st.altitude,
                st.climb_rate,
                st.q_dyn,
                cmd.alpha_cmd * 57.2958f,
                cmd.thrust_left,
                cmd.thrust_right
            );
            logFile.close();
        }
        xSemaphoreGive(sdMutex_);
    }
}

} // namespace raven