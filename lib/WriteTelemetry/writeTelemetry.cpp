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

void printTelemetry(const TelemetrySnapshot &t) {
    const raven::ActuatorCmd  &o = t.out;
    const raven::VehicleState &v = t.st;
    Serial.printf("%-10s %-7s %s fs=%-9s r=%6.1f p=%6.1f y=%6.1f agl=%6.1f vz=%+5.1f as=%4.1f a=%5.1f "
                  "thr=%.2f/%.2f nac=%5.1f/%5.1f srf=%+.2f/%+.2f/%+.2f xte=%+6.1f dtg=%7.0f gs=%4.1f "
                  "ter=%5.0f nav=%d mag=%d pit=%d %3.0fHz\n",
        raven::flightModeName(o.mode), raven::missionPhaseName(o.phase),
        o.armed ? "ARMED" : "safe ", raven::failsafeReasonName(o.fs_reason),
        v.roll * RAD_TO_DEG, v.pitch * RAD_TO_DEG, v.yaw * RAD_TO_DEG,
        v.agl, v.climb_rate, v.airspeed, o.alpha_cmd * RAD_TO_DEG,
        o.thrust_left, o.thrust_right, o.nacelle_left * RAD_TO_DEG, o.nacelle_right * RAD_TO_DEG,
        o.aileron, o.elevator, o.rudder,
        o.cross_track_dbg, v.mission_distance_to_go, v.ground_speed,
        v.terrain_valid ? v.terrain_elev_m : -1.0f,
        v.nav_valid ? 1 : 0, t.raw.mag_valid ? 1 : 0, t.raw.pitot_valid ? 1 : 0, t.loop_hz);
}

void printVehicleReport(const raven::FlightKinematics &fk, const raven::VehicleConfig &cfg,
                        uint32_t configIssues) {
    Serial.printf("Vehicle: %.3f kg, hover thrust %.0f%% of max, wing stall %.1f m/s, cruise %.1f m/s\n",
        cfg.mass, 100.0f * cfg.mass * cfg.g / (2.0f * cfg.thrust_max_per_rotor),
        fk.stallSpeed(1.225f), cfg.cruise_airspeed);
    for (uint32_t bit = 1; bit != 0 && bit <= configIssues; bit <<= 1) {
        if (configIssues & bit)
            Serial.printf("  CONFIG %s: %s\n", (bit & raven::CFG_FATAL_MASK) ? "FATAL" : "warning",
                          raven::configIssueText(bit));
    }
}
