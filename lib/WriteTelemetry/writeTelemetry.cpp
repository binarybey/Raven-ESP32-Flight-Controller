#include "writeTelemetry.h"

namespace raven {

bool TelemetryLogger::begin(SemaphoreHandle_t sdMutex) {
    sdMutex_ = sdMutex;
    if (xSemaphoreTake(sdMutex_, portMAX_DELAY) != pdTRUE) return false;

    bool ok = false;
    for (int i = 0; i < 1000 && !ok; ++i) {
        snprintf(filepath_, sizeof(filepath_), "/log%03d.csv", i);
        if (SD.exists(filepath_)) continue;
        file_ = SD.open(filepath_, FILE_WRITE);
        ok = static_cast<bool>(file_);
        break;
    }
    if (ok) {
        file_.println("time_ms,armed,mode,phase,failsafe,roll_deg,pitch_deg,yaw_deg,"
                      "alt_m,agl_m,climb_ms,q_dyn_pa,airspeed_ms,alpha_deg,"
                      "thr_L,thr_R,nac_L_deg,nac_R_deg,ail,ele,rud,"
                      "pos_x_m,pos_y_m,gs_ms,cog_deg,xte_m,seg_dtg_m,mission_dtg_m,nav_ok,terrain_m");
        file_.flush();
        lastFlushMs_ = millis();
    }
    xSemaphoreGive(sdMutex_);

    initialized_ = ok;
    return ok;
}

void TelemetryLogger::logState(const VehicleState& st, const ActuatorCmd& cmd) {
    if (!initialized_) return;

    // The terrain task holds the SD for one footprint lookup at a time
    // (a few ms); 50 ms still fits inside the 100 ms logging period.
    if (xSemaphoreTake(sdMutex_, 50 / portTICK_PERIOD_MS) == pdTRUE) {
        file_.printf("%lu,%d,%s,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,"
                     "%.3f,%.3f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.2f,%.1f,%.1f,%d,%.0f\n",
            (unsigned long)millis(),
            cmd.armed ? 1 : 0,
            flightModeName(cmd.mode),
            missionPhaseName(cmd.phase),
            failsafeReasonName(cmd.fs_reason),
            st.roll * 57.2958f,
            st.pitch * 57.2958f,
            st.yaw * 57.2958f,
            st.altitude,
            st.agl,
            st.climb_rate,
            st.q_dyn,
            st.airspeed,
            cmd.alpha_cmd * 57.2958f,
            cmd.thrust_left,
            cmd.thrust_right,
            cmd.nacelle_left * 57.2958f,
            cmd.nacelle_right * 57.2958f,
            cmd.aileron,
            cmd.elevator,
            cmd.rudder,
            st.pos_x,
            st.pos_y,
            st.ground_speed,
            st.course_over_ground * 57.2958f,
            cmd.cross_track_dbg,
            st.distance_to_go,
            st.mission_distance_to_go,
            st.nav_valid ? 1 : 0,
            st.terrain_valid ? st.terrain_elev_m : -1.0f
        );
        if (millis() - lastFlushMs_ >= 1000) {
            file_.flush();
            lastFlushMs_ = millis();
        }
        xSemaphoreGive(sdMutex_);
    }
}

} // namespace raven
