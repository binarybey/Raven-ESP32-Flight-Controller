#include "command_console.h"
#include "fcode_interpreter.h"

#include <string.h>
#include <ctype.h>

namespace raven {

static Command lookup(const char *s) {
    if (strcmp(s, "arm") == 0)    return Command::ARM;
    if (strcmp(s, "disarm") == 0) return Command::DISARM;
    if (strcmp(s, "land") == 0)   return Command::LAND;
    if (strcmp(s, "status") == 0) return Command::STATUS;
    if (strcmp(s, "help") == 0 || strcmp(s, "?") == 0) return Command::HELP;
    return Command::UNKNOWN;
}

Command CommandConsole::poll() {
    while (io_.available() > 0) {
        const int c = io_.read();
        if (c < 0) break;
        if (c == '\r' || c == '\n') {
            if (n_ == 0 && !overflow_) continue;   // blank line / second half of CRLF
            while (n_ > 0 && (buf_[n_-1] == ' ' || buf_[n_-1] == '\t')) --n_;
            buf_[n_] = '\0';
            const bool was_overflow = overflow_;
            n_ = 0;
            overflow_ = false;
            return was_overflow ? Command::UNKNOWN : lookup(buf_);
        }
        if (n_ < sizeof(buf_) - 1) {
            if (!(n_ == 0 && c == ' ')) buf_[n_++] = (char)tolower(c);
        } else {
            overflow_ = true;
        }
    }
    return Command::NONE;
}

const char *commandHelpText() {
    return "Commands:\n"
           "  arm     - pre-arm checks, then arm (with a mission: TAKEOFF)\n"
           "  disarm  - MOTORS OFF IMMEDIATELY (also in flight)\n"
           "  land    - controlled landing at the current position\n"
           "  status  - status summary\n"
           "  help    - this text\n";
}

ConsoleReply executeCommand(FlightKinematics &fk, Command cmd) {
    ConsoleReply rep = { false, "unsupported" };
    switch (cmd) {
        case Command::ARM:
            rep.ok = fk.arm(&rep.msg);
            break;
        case Command::DISARM:
            fk.disarm();
            rep.ok  = true;
            rep.msg = "disarmed - motors off";
            break;
        case Command::LAND:
            if (!fk.armed()) {
                rep.msg = "not armed";
            } else {
                fk.requestFailsafe(FailsafeReason::COMMANDED);
                rep.ok  = true;
                rep.msg = "landing at current position";
            }
            break;
        default:
            break;
    }
    return rep;
}

void printStatus(Stream &io, const TelemetrySnapshot &t, bool terrainReady, bool routeCovered) {
    const VehicleState &st = t.st;
    const RawSensors   &r  = t.raw;
    const fcode::MissionInfo &mi = fcode::info();
    io.printf("armed=%d mode=%s phase=%s failsafe=%s loop=%.0fHz\n",
        t.out.armed ? 1 : 0, flightModeName(t.out.mode), missionPhaseName(t.out.phase),
        failsafeReasonName(t.out.fs_reason), t.loop_hz);
    io.printf("imu=%d mag=%d ahrs_ready=%d baro=%d pitot=%d gnss_fix=%d sats=%d hdop=%.1f nav=%d\n",
        r.imu_valid, r.mag_valid, r.ahrs_ready, r.baro_valid, r.pitot_valid,
        r.gnss_fix_valid, r.gnss_sats, r.gnss_hdop, st.nav_valid ? 1 : 0);
    io.printf("mission: %s, %d segments, %.0f m total, %.0f m to go, clearance %.0f m\n",
        mi.ok ? "loaded" : "none", mi.segment_count, mi.total_length_m,
        st.mission_distance_to_go, mi.ground_clearance_m);
    io.printf("attitude r=%.1f p=%.1f y(true)=%.1f  agl=%.1f alt=%.1f\n",
        st.roll * RAD_TO_DEG, st.pitch * RAD_TO_DEG, st.yaw * RAD_TO_DEG, st.agl, st.altitude);
    io.printf("terrain: dem=%s route=%s now=%d elev=%.0f m lookup=%lu ms\n",
        terrainReady ? "ok" : "MISSING", routeCovered ? "covered" : "NOT covered",
        st.terrain_valid ? 1 : 0, st.terrain_elev_m, (unsigned long)t.terrain_ms);
}

}  // namespace raven
