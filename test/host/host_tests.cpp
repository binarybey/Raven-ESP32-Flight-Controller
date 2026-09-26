// Host-side tests for the pure (Arduino-free) parts of the AHRS firmware:
// NMEA parser, F-Code interpreter, DEM terrain reader, FlightKinematics
// mode/mission/failsafe logic. The terrain tests read the real tiles in
// test/vtol_bin_tiles (not in git) and are skipped if they're absent.
// Run with test/host/run_host_tests.sh. These drive the REAL firmware code
// with scripted sensor inputs - no vehicle dynamics are simulated, so they
// check decisions (modes, phases, failsafes, geometry), not flight quality.
#include <cstdio>
#include <cmath>
#include <cstring>
#include <string>
#include <functional>
#include <filesystem>
#include <fstream>

#include "nmea_parse.h"
#include "fcode_interpreter.h"
#include "flight_kinematics.h"
#include "terrain.h"

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { ++g_pass; } else { ++g_fail; \
    std::printf("  FAIL %s:%d: %s  -- ", __FILE__, __LINE__, #cond); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)
#define NEAR(a, b, tol) (std::fabs((double)(a) - (double)(b)) <= (tol))

static const double kPi = 3.14159265358979;

// ---------------------------------------------------------------------------
// NMEA
// ---------------------------------------------------------------------------
static std::string withCs(const std::string &body) {
    unsigned cs = 0;
    for (char c : body) cs ^= (unsigned char)c;
    char buf[8];
    std::snprintf(buf, sizeof buf, "*%02X", cs);
    return body + buf;
}

static void testNmea() {
    std::printf("NMEA parser\n");
    nmea::Sentence s;

    // GN talker, 5 decimals
    auto r = nmea::parse(withCs("GNGGA,123519.00,3336.55001,N,04430.00000,E,1,09,0.9,845.4,M,-2.0,M,,").c_str(), s);
    CHECK(r == nmea::Result::OK && s.type == nmea::Type::GGA, "GNGGA not parsed (r=%d)", (int)r);
    CHECK(NEAR(s.gga.lat.deg, 33 + 36.55001/60.0, 1e-9), "lat %.9f", s.gga.lat.deg);
    CHECK(NEAR(s.gga.lon.deg, 44.5, 1e-9), "lon %.9f", s.gga.lon.deg);
    CHECK(s.gga.fix_quality == 1 && s.gga.sats_used == 9 && NEAR(s.gga.altitude_msl_m, 845.4, 1e-3), "fields");

    // GP talker, 4 decimals (old parser: ~917 m error)
    r = nmea::parse(withCs("GPGGA,123519,3336.5500,N,04430.0000,E,1,08,1.1,845.4,M,,M,,").c_str(), s);
    CHECK(r == nmea::Result::OK, "4-decimal GGA r=%d", (int)r);
    CHECK(NEAR(s.gga.lat.deg, 33 + 36.55/60.0, 1e-9), "4-decimal lat %.9f", s.gga.lat.deg);
    CHECK(NEAR(s.gga.lat.s, 33.0, 1e-3), "DMS seconds %.4f (expected 33.0)", s.gga.lat.s);

    // South/West signs
    r = nmea::parse(withCs("GPGGA,000000,3336.5500,S,04430.0000,W,1,08,1.1,5.0,M,,M,,").c_str(), s);
    CHECK(r == nmea::Result::OK && s.gga.lat.deg < 0 && s.gga.lon.deg < 0, "S/W signs");

    // Bad checksum / missing checksum
    std::string bad = withCs("GNGGA,123519.00,3336.55001,N,04430.00000,E,1,09,0.9,845.4,M,-2.0,M,,");
    bad[10] = '9';
    CHECK(nmea::parse(bad.c_str(), s) == nmea::Result::BAD_CHECKSUM, "corrupted sentence accepted");
    CHECK(nmea::parse("GNGGA,123519.00,3336.55001,N", s) == nmea::Result::BAD_CHECKSUM, "no checksum accepted");

    // No fix: position fields empty -> OK, quality 0
    r = nmea::parse(withCs("GNGGA,123519.00,,,,,0,00,99.99,,,,,,").c_str(), s);
    CHECK(r == nmea::Result::OK && s.gga.fix_quality == 0, "no-fix GGA r=%d", (int)r);

    // RMC with / without course
    r = nmea::parse(withCs("GNRMC,123519.00,A,3336.55001,N,04430.00000,E,9.72,087.5,250926,,,A").c_str(), s);
    CHECK(r == nmea::Result::OK && s.type == nmea::Type::RMC && s.rmc.active, "RMC r=%d", (int)r);
    CHECK(NEAR(s.rmc.speed_knots, 9.72, 1e-4) && s.rmc.has_course && NEAR(s.rmc.course_deg, 87.5, 1e-4), "RMC fields");
    CHECK(s.rmc.day == 25 && s.rmc.month == 9 && s.rmc.year == 26, "RMC date");
    r = nmea::parse(withCs("GNRMC,123519.00,A,3336.55001,N,04430.00000,E,0.02,,250926,,,A").c_str(), s);
    CHECK(r == nmea::Result::OK && !s.rmc.has_course, "RMC empty course");

    // GSV + ignored sentences
    r = nmea::parse(withCs("GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00").c_str(), s);
    CHECK(r == nmea::Result::OK && s.gsv.count == 4 && s.gsv.sats[3].id == 13 && s.gsv.sats_in_view == 11, "GSV");
    CHECK(nmea::parse(withCs("GNGSA,A,3,01,02,,,,,,,,,,,1.5,0.9,1.2").c_str(), s) == nmea::Result::IGNORED, "GSA not ignored");
    CHECK(nmea::parse(withCs("PUBX,00,123519").c_str(), s) == nmea::Result::IGNORED, "proprietary not ignored");
}

// ---------------------------------------------------------------------------
// F-Code
// ---------------------------------------------------------------------------
// Origin 33deg30'00"N 44deg30'00"E. Lines: 20" north, 20" east. Then a CW
// quarter arc centred 20" south of the arc start, ending 24" east of centre.
static const char *kMission =
    "%\n"
    "F43; Z50;\r\n"
    "F90; X0443000; Y333000;\n"
    "F01; X+0.0; Y+20.0;\n"
    "F01; X+20.0; Y+0.0;\n"
    "F02; I0443020; J333000; R617; A90; X0443044; Y333000;\n"
    "F39;\n"
    "%\n";

static const double kLat0 = 33.5, kLon0 = 44.5;
static const double kMLat = 30.87 * 3600.0;
static double mLon() { return 30.87 * 3600.0 * std::cos(kLat0 * kPi / 180.0); }

static void testFcode() {
    std::printf("F-Code interpreter\n");
    CHECK(fcode::begin(kMission), "parse failed: %s (line %d)", fcode::info().error, fcode::info().error_line);
    const auto &mi = fcode::info();
    const double L1 = 20 * 30.87, L2 = 20 * 30.87 * std::cos(kLat0*kPi/180), R = 20 * 30.87;
    const double arcLen = R * kPi / 2;
    CHECK(mi.segment_count == 3, "segments %d", mi.segment_count);
    CHECK(NEAR(mi.total_length_m, L1 + L2 + arcLen, 2.0), "total %.1f vs %.1f", mi.total_length_m, L1 + L2 + arcLen);
    CHECK(mi.arc_warnings == 0, "arc warnings %d", mi.arc_warnings);

    fcode::NavOutput nav;
    // On line 1, 10 m east (right) of the path
    fcode::update(10.0f, 300.0f, nav);
    CHECK(nav.en_route && nav.segment_index == 0, "seg %d", nav.segment_index);
    CHECK(NEAR(nav.cross_track_error, 10.0, 1e-3), "xte %.3f", nav.cross_track_error);
    CHECK(NEAR(nav.distance_to_go, L1 - 300, 0.1), "dtg %.2f", nav.distance_to_go);
    CHECK(NEAR(nav.mission_distance_to_go, (L1 - 300) + L2 + arcLen, 2.0), "mission dtg %.1f", nav.mission_distance_to_go);

    // Finding #5: 10 m off to the side at the end of the line - old code (5 m
    // radius) would never complete. Progress-based completion must.
    fcode::update(10.0f, (float)L1 - 0.5f, nav);
    CHECK(nav.segment_completed, "line not completed with 10 m cross-track");
    // Finding #2: mission distance continuous across the boundary
    const float before = nav.mission_distance_to_go;
    fcode::update(10.0f, (float)L1 - 0.4f, nav);
    CHECK(nav.segment_index == 1, "seg %d", nav.segment_index);
    CHECK(std::fabs(nav.mission_distance_to_go - before) < 12.0f, "mission dtg jumped %.1f -> %.1f", before, nav.mission_distance_to_go);
    CHECK(nav.mission_distance_to_go > 400.0f, "mission dtg %.1f should stay large at a segment end", nav.mission_distance_to_go);

    // Walk line 2 to its end, then along the CW arc.
    fcode::update((float)L2 - 0.2f, (float)L1, nav);
    CHECK(nav.segment_completed, "line 2 not completed");
    const double cx = L2, cy = 0;
    float prev = 1e9f; bool monotonic = true, completed = false;
    for (int deg = 1; deg <= 92 && !completed; ++deg) {
        const double b = deg * kPi / 180;
        fcode::update((float)(cx + R*std::sin(b)), (float)(cy + R*std::cos(b)), nav);
        if (nav.distance_to_go > prev + 1e-3f) monotonic = false;
        prev = nav.distance_to_go;
        if (deg == 45) {
            CHECK(NEAR(nav.desired_course_rad, 135*kPi/180, 1e-3), "arc course %.4f", nav.desired_course_rad);
            CHECK(NEAR(nav.distance_to_go, R * kPi / 4, 2.0), "arc dtg %.1f", nav.distance_to_go);
        }
        completed = nav.mission_complete;
    }
    CHECK(monotonic, "arc distance_to_go not monotonic");
    CHECK(completed && fcode::complete(), "arc/mission not completed");
    // Final hold geometry: CW arc ending at bearing 90 -> final course 180 (south)
    const double ex = cx + 617.77, ey = 0;   // 24" east of the centre
    fcode::update((float)ex + 1.0f, (float)ey + 5.0f, nav);
    CHECK(!nav.en_route && nav.mission_complete, "hold flags");
    CHECK(NEAR(std::fabs(nav.desired_course_rad), kPi, 1e-3), "final course %.4f", nav.desired_course_rad);
    CHECK(NEAR(nav.mission_distance_to_go, 5.0, 0.6), "hold along %.2f (point is 5 m 'ahead' going south)", nav.mission_distance_to_go);
    CHECK(NEAR(nav.cross_track_error, -1.0, 0.6), "hold xte %.2f (east of a southbound line = left)", nav.cross_track_error);

    // Overshoot jump on an arc: old code wrapped to ~360 deg and circled.
    CHECK(fcode::begin(kMission), "reparse");
    fcode::update(0, 1, nav); fcode::update(0, (float)L1, nav);      // complete line 1
    fcode::update((float)L2, (float)L1, nav);                          // complete line 2
    const double b85 = 85 * kPi / 180, b100 = 100 * kPi / 180;
    fcode::update((float)(cx + R*std::sin(b85)), (float)(cy + R*std::cos(b85)), nav);
    CHECK(nav.en_route && nav.segment_index == 2, "on arc");
    fcode::update((float)(cx + R*std::sin(b100)), (float)(cy + R*std::cos(b100)), nav);
    CHECK(nav.mission_complete, "15-deg jump past the arc end did not complete (dtg %.1f)", nav.distance_to_go);

    // Full circle (end == start), starting slightly BEHIND the start point.
    const char *circle =
        "F43; Z30;\nF90; X0443000; Y333000;\n"
        "F03; I0443000; J332940; R617; A360; X0443000; Y333000;\nF39;\n";
    CHECK(fcode::begin(circle), "circle parse: %s", fcode::info().error);
    const double Rc = 20 * 30.87;
    CHECK(NEAR(fcode::info().total_length_m, 2*kPi*Rc, 2.0), "circle length %.1f", fcode::info().total_length_m);
    // CCW around centre (0, -Rc): start at bearing 0; CCW = decreasing bearing.
    bool early = false;
    for (int deg = 1; deg >= -359; --deg) {     // deg=+1 is 1 deg "behind" the start
        const double b = deg * kPi / 180;
        fcode::update((float)(Rc*std::sin(b)), (float)(-Rc + Rc*std::cos(b)), nav);
        if (nav.mission_complete && deg > -355) early = true;
    }
    CHECK(!early, "full circle completed early");
    // -359 deg is 1 deg short of closing the circle; -361 deg (= -1) is 1 deg past it
    fcode::update((float)(Rc*std::sin(-1*kPi/180)), (float)(-Rc + Rc*std::cos(-1*kPi/180)), nav);
    CHECK(nav.mission_complete, "full circle not completed");

    // Path sampling (terrain look-ahead / route check) and inverse projection
    CHECK(fcode::begin(kMission), "reparse for pathPointAt");
    {
        float px, py;
        CHECK(fcode::pathPointAt(0, px, py) && NEAR(px, 0, 1e-3) && NEAR(py, 0, 1e-3), "s=0 (%.2f,%.2f)", px, py);
        fcode::pathPointAt((float)L1, px, py);
        CHECK(NEAR(px, 0, 0.01) && NEAR(py, L1, 0.01), "s=L1 (%.2f,%.2f)", px, py);
        fcode::pathPointAt((float)(L1 + L2/2), px, py);
        CHECK(NEAR(px, L2/2, 0.05) && NEAR(py, L1, 0.05), "mid line 2 (%.2f,%.2f)", px, py);
        fcode::pathPointAt((float)(L1 + L2 + arcLen/2), px, py);   // bearing 45 deg on the arc
        CHECK(NEAR(px, cx + R*std::sin(kPi/4), 0.5) && NEAR(py, R*std::cos(kPi/4), 0.5), "arc mid (%.2f,%.2f)", px, py);
        fcode::pathPointAt(1e6f, px, py);                          // clamped to the end
        CHECK(NEAR(px, cx + R, 1.0) && NEAR(py, 0, 1.0), "clamped end (%.2f,%.2f)", px, py);
        double lat, lon; float bx, by;
        fcode::localToGps(1234.5f, -678.9f, lat, lon);
        fcode::gpsToLocal(lat, lon, bx, by);
        CHECK(NEAR(bx, 1234.5, 0.01) && NEAR(by, -678.9, 0.01), "local<->gps round trip (%.3f,%.3f)", bx, by);
    }

    // Validation failures (finding #9)
    auto fails = [](const char *txt, const char *what) {
        const bool ok = fcode::begin(txt);
        CHECK(!ok && !fcode::loaded(), "%s was accepted", what);
        if (!ok) std::printf("    rejected (%s): %s\n", what, fcode::info().error);
    };
    fails("F43; Z50;\nF90; X0443000; Y333000;\nF01; X+0.0; Y+20.0;\n", "missing F39 (truncated file)");
    fails("F90; X0443000; Y333000;\nF01; X+0.0; Y+20.0;\nF39;\n", "missing F43");
    fails("F43; Z50;\nF90; X-0443000; Y333000;\nF01; X+0.0; Y+20.0;\nF39;\n", "negative DMS");
    fails("F43; Z50;\nF90; X0446000; Y333000;\nF01; X+0.0; Y+20.0;\nF39;\n", "minutes >= 60");
    fails("F43; Z50;\nF90; X0443000; Y333000;\nF01; X+abc; Y+20.0;\nF39;\n", "non-numeric delta");
    fails("F43; Z50;\nF90; X0443000; Y333000;\nF39;\n", "no segments");
    std::string longLine = "F43; Z50;\nF90; X0443000; Y333000;\nF01; X+0.0; Y+20.0;";
    longLine += std::string(200, ' ') + "\nF39;\n";
    fails(longLine.c_str(), "line too long");
}

// ---------------------------------------------------------------------------
// FlightKinematics
// ---------------------------------------------------------------------------
struct Sim {
    raven::FlightKinematics fk;
    raven::RawSensors raw;
    raven::VehicleState st;
    raven::ActuatorCmd out;
    raven::Setpoints manual;
    double x = 0, y = 0;          // local position (m)
    double vx = 0, vy = 0;        // m/s
    double t = 0;
    int cyc = 0;

    Sim(bool withPitot = false, double pitotQ = 0) {
        raven::VehicleConfig cfg;
        raven::ControlGains gains;
        fk.begin(cfg, gains);
        fk.setMissionTerrainOk(true);        // boot route check passed
        raw.terrain_valid  = true;           // flat ground at 1000 m MSL
        raw.terrain_elev_m = 1000.0f;
        raw.imu_valid = raw.mag_valid = raw.ahrs_ready = true;
        raw.baro_valid = true;
        raw.baro_altitude = 1000.0f;
        raw.baro_pressure_pa = 90000.0f;
        raw.baro_temperature_c = 20.0f;
        raw.gnss_fix_valid = true;
        raw.gnss_altitude_m = 1000.0f;
        raw.gnss_hdop = 0.8f;
        raw.gnss_sats = 10;
        raw.gnss_vel_valid = true;
        raw.pitot_valid = withPitot;
        raw.pitot_q_pa = (float)pitotQ;
        feedGnss();
    }
    // Vehicle MSL altitude, seen consistently by baro (+ its offset) and GNSS.
    float baroOffset = 0.0f;
    void setAlt(float msl) { raw.baro_altitude = msl + baroOffset; raw.gnss_altitude_m = msl; }
    void feedGnss() {
        raw.gnss_lat_deg = kLat0 + y / kMLat;
        raw.gnss_lon_deg = kLon0 + x / mLon();
        raw.gnss_ground_speed = (float)std::hypot(vx, vy);
        raw.gnss_course_rad = (float)std::atan2(vx, vy);
        raw.gnss_fix_seq++;
        raw.gnss_vel_seq++;
    }
    // 100 Hz; GNSS fix + velocity at 1 Hz unless gnssOn == false
    void run(double seconds, bool gnssOn = true, std::function<void()> each = nullptr) {
        const int n = (int)std::lround(seconds * 100);
        for (int i = 0; i < n; ++i) {
            x += vx * 0.01; y += vy * 0.01; t += 0.01;
            if (gnssOn && (++cyc % 100) == 0) feedGnss();
            out = fk.runCycle(0.01f, raw, manual, &st);
            if (each) each();
        }
    }
};

static void testFlightKinematics() {
    std::printf("FlightKinematics\n");

    // --- config validation ---
    {
        raven::FlightKinematics fk;
        raven::VehicleConfig cfg; raven::ControlGains g;
        const uint32_t issues = fk.begin(cfg, g);
        CHECK((issues & raven::CFG_FATAL_MASK) == 0, "default config has fatal issues 0x%x", issues);
        std::printf("    default config issues: 0x%x, stall %.1f m/s, hover %.0f%%\n", issues,
                    fk.stallSpeed(1.225f), 100*cfg.mass*cfg.g/(2*cfg.thrust_max_per_rotor));
        cfg.mass = 4.0f;
        CHECK((fk.begin(cfg, g) & raven::CFG_HOVER_THRUST) != 0, "4 kg on 16 N rotors not flagged");
    }

    // --- declination (+ NWU sign flip) ---
    {
        fcode::begin(nullptr);
        Sim s;
        s.raw.fusion_yaw_deg = 0.0f;           // magnetic north
        s.run(0.05);
        CHECK(NEAR(s.st.yaw * 180 / kPi, 6.1436, 0.01), "true yaw %.3f", s.st.yaw * 180 / kPi);
        s.raw.fusion_yaw_deg = -90.0f;         // NWU: nose right 90 = magnetic east
        s.run(0.05);
        CHECK(NEAR(s.st.yaw * 180 / kPi, 90 + 6.1436, 0.01), "true yaw %.3f", s.st.yaw * 180 / kPi);
    }

    // --- pre-arm checks + arming (finding #8) ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s;
        const char *why = nullptr;
        CHECK(!s.fk.arm(&why), "armed with no data");
        s.raw.ahrs_ready = false; s.run(2);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "AHRS"), "ahrs: %s", why);
        s.raw.ahrs_ready = true; s.raw.fusion_roll_deg = 20; s.run(0.1);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "level"), "level: %s", why);
        s.raw.fusion_roll_deg = 0; s.raw.gnss_fix_valid = false; s.run(6, false);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "GNSS"), "gnss: %s", why);
        s.raw.gnss_fix_valid = true; s.x = 200; s.run(2);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "mission start"), "far from F90: %s", why);
        s.x = 0; s.run(6);
        s.fk.setMissionTerrainOk(false);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "DEM tiles missing"), "route: %s", why);
        s.fk.setMissionTerrainOk(true);
        s.raw.terrain_valid = false; s.run(0.1);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "no DEM"), "dem here: %s", why);
        s.raw.terrain_valid = true; s.run(0.1);
        CHECK(!s.fk.arm(&why) && std::strstr(why, "settling"), "settle: %s", why);
        s.run(25);
        CHECK(!s.fk.armed() && s.out.mode == raven::FlightMode::DISARMED, "not disarmed at boot");
        CHECK(s.out.thrust_left == 0.0f, "thrust while disarmed");
        CHECK(s.fk.arm(&why), "arm refused: %s", why);
        s.run(0.05);
        CHECK(s.fk.phase() == raven::MissionPhase::TAKEOFF, "phase %s", raven::missionPhaseName(s.fk.phase()));
    }

    // --- finding #1: no pitot -> never converts, even far from the end ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s(false);
        s.run(31);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.setAlt(1050); s.run(15);            // climb above 90% of 50 m clearance
        CHECK(s.fk.phase() == raven::MissionPhase::ENROUTE, "phase %s", raven::missionPhaseName(s.fk.phase()));
        s.vy = 5.0; float maxAlpha = 0; bool leftHover = false;
        s.run(60, true, [&]{ maxAlpha = std::fmax(maxAlpha, s.out.alpha_cmd);
                             if (s.out.mode != raven::FlightMode::HOVER) leftHover = true; });
        CHECK(!leftHover && maxAlpha == 0.0f, "converted without pitot (max alpha %.1f deg)", maxAlpha * 57.3);
        CHECK(s.st.mission_distance_to_go > 400, "mission dtg %.0f (test premise)", s.st.mission_distance_to_go);
    }

    // --- finding #2: with pitot, no reconversion at segment ends ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s(true, 450.0);                               // q = 450 Pa ~ 29 m/s
        s.run(31);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.setAlt(1050); s.run(15);
        s.vy = 1.0; s.run(3);
        CHECK(s.out.mode == raven::FlightMode::TRANS_FWD || s.out.mode == raven::FlightMode::FORWARD,
              "did not convert with pitot, mode %s", raven::flightModeName(s.out.mode));
        // fly the path at 25 m/s: north leg, then east leg
        bool backEarly = false; float dtgAtBack = -1;
        auto watch = [&]{
            if (s.out.mode == raven::FlightMode::TRANS_BACK && dtgAtBack < 0) dtgAtBack = s.st.mission_distance_to_go;
            if (s.out.mode == raven::FlightMode::TRANS_BACK && s.st.mission_distance_to_go > 260) backEarly = true;
        };
        s.vx = 0; s.vy = 25; s.run(24.8, true, watch);            // north leg, through its end
        s.x = 0; s.y = 20 * 30.87; s.vx = 25; s.vy = 0; s.run(20.5, true, watch);   // east leg, through its end
        CHECK(!backEarly, "reconverted at a segment end");
        CHECK(dtgAtBack < 0, "TRANS_BACK already at mission dtg %.0f (arc still ~970 m)", dtgAtBack);
        // along the arc (CW around (L2, 0)) at 25 m/s until inside dist_to_hover
        const double L2 = 20 * 30.87 * std::cos(kLat0*kPi/180), R = 20 * 30.87;
        s.vx = 0; s.vy = 0;
        for (double deg = 0; deg <= 85 && dtgAtBack < 0; deg += 25.0 / R * 180 / kPi) {
            const double b = deg * kPi / 180;
            s.x = L2 + R*std::sin(b); s.y = R*std::cos(b);
            s.vx = 25*std::cos(b); s.vy = -25*std::sin(b);
            s.feedGnss(); s.run(1.0, false, watch);
        }
        CHECK(!backEarly, "reconverted too early on the arc");
        CHECK(dtgAtBack > 0 && dtgAtBack <= 251, "TRANS_BACK at mission dtg %.0f", dtgAtBack);
        std::printf("    TRANS_BACK began at mission distance-to-go %.0f m\n", dtgAtBack);
    }

    // --- finding #3 (no-DEM fallback): height above the arming point is
    //     immune to baro-bias convergence ---
    {
        fcode::begin(nullptr);
        Sim s;
        s.raw.terrain_valid = false;
        s.raw.baro_altitude = 1000.0f + 14.3f;           // P0-biased baro
        s.raw.gnss_altitude_m = 1000.0f;                 // truth
        s.run(1);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.run(0.1);
        const float alt0 = s.st.altitude, agl0 = s.st.agl;
        float worst = 0;
        s.run(600, true, [&]{ worst = std::fmax(worst, std::fabs(s.st.agl - agl0)); });
        std::printf("    fused alt moved %.1f m while AGL moved at most %.2f m\n", alt0 - s.st.altitude, worst);
        CHECK(alt0 - s.st.altitude > 10.0f, "test premise: bias estimate should converge");
        CHECK(worst < 0.5f, "AGL drifted %.2f m", worst);
    }

    // --- finding #6: lateral velocity from course over ground ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s;
        s.run(31);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.vx = 2.0; s.vy = 0.0;                            // drifting EAST, nose north
        s.run(1.5);
        CHECK(NEAR(s.out.lateral_vel_est_dbg, 2.0, 0.05), "lateral vel %.2f (heading-based would read 0)", s.out.lateral_vel_est_dbg);
        CHECK(NEAR(s.out.along_vel_est_dbg, 0.0, 0.05), "along vel %.2f", s.out.along_vel_est_dbg);
    }

    // --- finding #8: nav loss -> hover, wait, then land ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s;
        s.run(31);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.setAlt(1050); s.run(15);
        s.raw.terrain_valid = false;                     // no position -> no DEM lookups either;
        s.run(3.9, false);                               // NAV_LOST must govern, not TERRAIN
        CHECK(s.st.nav_valid, "nav invalid too early");
        s.run(0.3, false);
        CHECK(!s.st.nav_valid, "nav still valid after stale timeout");
        CHECK(s.fk.mode() != raven::FlightMode::FAILSAFE, "failsafe too early");
        s.run(19.5, false);
        CHECK(s.fk.mode() != raven::FlightMode::FAILSAFE, "failsafe before nav_loss_land_timeout_s");
        s.run(1.0, false);
        CHECK(s.fk.mode() == raven::FlightMode::FAILSAFE && s.fk.failsafeReason() == raven::FailsafeReason::NAV_LOST,
              "mode %s reason %s", raven::flightModeName(s.fk.mode()), raven::failsafeReasonName(s.fk.failsafeReason()));
        CHECK(s.out.thrust_left > 0.0f, "failsafe cut the motors");
    }

    // --- DEM clearance: AGL = GNSS-calibrated MSL - terrain, whatever the baro offset ---
    {
        fcode::begin(nullptr);
        Sim s;
        s.baroOffset = 14.3f;                            // P0-biased baro
        s.raw.terrain_elev_m = 950.0f;                   // terrain 950 m, vehicle at 1000 m MSL
        s.setAlt(1000.0f);                               // -> 50 m clearance
        s.run(31);                                       // disarmed: ground tau 10 s, 3 taus
        CHECK(s.st.terrain_valid && NEAR(s.st.agl, 50.0, 1.5), "DEM AGL %.2f (expected 50)", s.st.agl);
        s.raw.terrain_elev_m = 1020.0f;                  // ridge ahead enters the look-ahead
        s.run(0.05);
        CHECK(NEAR(s.st.agl, -20.0, 1.5), "AGL vs ridge %.2f (expected -20)", s.st.agl);
    }

    // --- DEM lost while the position is still known -> hold MSL, then land ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s; s.run(31);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.setAlt(1050); s.run(15);
        CHECK(s.fk.phase() == raven::MissionPhase::ENROUTE, "phase %s", raven::missionPhaseName(s.fk.phase()));
        s.raw.terrain_valid = false;
        s.run(4.9);
        CHECK(s.fk.mode() != raven::FlightMode::FAILSAFE, "TERRAIN failsafe too early");
        s.run(0.3);
        CHECK(s.fk.failsafeReason() == raven::FailsafeReason::TERRAIN, "reason %s",
              raven::failsafeReasonName(s.fk.failsafeReason()));
    }

    // --- nacelles in hover: aft travel for nose-up pitch, differential for
    //     yaw, and both at once without leaving the mechanical range ---
    {
        fcode::begin(nullptr);
        const raven::VehicleConfig cfg;
        auto inRange = [&](const raven::ActuatorCmd &o) {
            return o.nacelle_left  >= cfg.nacelle_min_rad - 1e-4f && o.nacelle_left  <= cfg.nacelle_max_rad + 1e-4f &&
                   o.nacelle_right >= cfg.nacelle_min_rad - 1e-4f && o.nacelle_right <= cfg.nacelle_max_rad + 1e-4f;
        };
        Sim s; s.run(3);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        bool ok = true;

        // Nose 20 deg DOWN (Fusion pitch is nose-down positive): the stabiliser
        // wants nose-up, which in hover means both nacelles tilting AFT.
        s.raw.fusion_pitch_deg = 20.0f;
        float minNac = 1.0f;
        s.run(1.0, true, [&]{ minNac = std::fmin(minNac, std::fmin(s.out.nacelle_left, s.out.nacelle_right));
                              ok = ok && inRange(s.out); });
        CHECK(minNac < -0.11f, "nacelles never tilted past the old -5.7 deg clip (min %.1f deg)", minNac * 57.3);
        CHECK(ok, "pitch: a nacelle left its mechanical range");

        // Level again, yawing LEFT at 60 deg/s (Fusion z is up): the rate loop
        // wants yaw right = left nacelle forward, right nacelle aft.
        s.raw.fusion_pitch_deg = 0.0f; s.run(1.0);
        s.raw.fusion_gyro_z_dps = 60.0f;
        float maxDiff = 0.0f, minRight = 1.0f;
        s.run(0.5, true, [&]{ maxDiff  = std::fmax(maxDiff, s.out.nacelle_left - s.out.nacelle_right);
                              minRight = std::fmin(minRight, s.out.nacelle_right);
                              ok = ok && inRange(s.out); });
        CHECK(maxDiff > 0.15f && minRight < -0.05f, "yaw: diff %.1f deg, right min %.1f deg",
              maxDiff * 57.3, minRight * 57.3);
        CHECK(ok, "yaw: a nacelle left its mechanical range");

        // Both at once: the aft-going nacelle hits -15 deg, yaw must survive
        // through the other one instead of being clipped away.
        s.raw.fusion_pitch_deg = 20.0f;
        float diffBoth = 0.0f;
        s.run(0.5, true, [&]{ diffBoth = s.out.nacelle_left - s.out.nacelle_right; ok = ok && inRange(s.out); });
        CHECK(ok, "pitch+yaw: a nacelle left its mechanical range");
        CHECK(diffBoth > 0.05f, "pitch+yaw: yaw differential lost (%.1f deg)", diffBoth * 57.3);
        std::printf("    hover nacelles: pitch-down min %.1f deg, yaw diff %.1f deg, pitch+yaw diff %.1f deg\n",
                    minNac * 57.3, maxDiff * 57.3, diffBoth * 57.3);
    }

    // --- sensor failure: IMU ---
    {
        fcode::begin(nullptr);
        Sim s; s.run(3);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.raw.imu_valid = false; s.run(0.05);
        CHECK(s.fk.mode() != raven::FlightMode::FAILSAFE, "IMU failsafe too early");
        s.run(0.1);
        CHECK(s.fk.failsafeReason() == raven::FailsafeReason::IMU, "reason %s", raven::failsafeReasonName(s.fk.failsafeReason()));
    }

    // --- touchdown detection ---
    {
        fcode::begin(nullptr);
        Sim s; s.run(3);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.fk.requestFailsafe(raven::FailsafeReason::COMMANDED);
        // Descent start: thrust drops before the lagging 2 Hz baro climb-rate
        // estimate moves - must NOT look like touchdown (mid-air disarm).
        s.raw.baro_climb_rate = 0.0f;
        s.run(4.9);
        CHECK(s.fk.armed(), "disarmed during the climb-rate lag at descent start");
        s.raw.baro_climb_rate = -0.8f;                   // descending through the air
        s.run(30);
        CHECK(s.fk.armed(), "disarmed while still descending");
        s.raw.baro_climb_rate = 0.0f;                    // on the ground
        double tDisarm = -1; const double t0 = s.t;
        s.run(15, true, [&]{ if (tDisarm < 0 && !s.fk.armed()) tDisarm = s.t - t0; });
        CHECK(tDisarm > 0 && tDisarm < 10, "touchdown not detected (t=%.1f)", tDisarm);
        std::printf("    touchdown -> disarm after %.1f s\n", tDisarm);
        CHECK(s.out.thrust_left == 0.0f && !s.out.armed, "outputs not off after touchdown");
    }

    // --- full mission lifecycle ---
    {
        CHECK(fcode::begin(kMission), "mission");
        Sim s; s.run(31);
        const char *why; CHECK(s.fk.arm(&why), "arm: %s", why);
        s.run(1);
        CHECK(s.fk.phase() == raven::MissionPhase::TAKEOFF, "takeoff");
        CHECK(s.out.along_vel_sp_dbg == 0.0f || std::fabs(s.out.along_vel_sp_dbg) < 0.5f, "moving during takeoff");
        s.setAlt(1050); s.run(15);
        CHECK(s.fk.phase() == raven::MissionPhase::ENROUTE, "enroute");
        // Teleport along the path (perfect tracking), 1 fix per second
        const double L1 = 20 * 30.87, L2 = 20 * 30.87 * std::cos(kLat0*kPi/180), R = 20*30.87;
        s.vy = 5; s.run(L1 / 5 + 0.5); s.x = 0; s.y = L1;
        s.vy = 0; s.vx = 5; s.run(L2 / 5 + 0.5); s.x = L2; s.y = L1;
        s.vx = 0;
        for (int deg = 1; deg <= 91; ++deg) {
            const double b = deg * kPi / 180;
            s.x = L2 + R*std::sin(b); s.y = R*std::cos(b); s.feedGnss(); s.run(0.5, false);
        }
        s.x = L2 + 617.77; s.y = 0;                     // park exactly on the final point
        s.run(5);                                        // estimate converges (1 Hz fixes, tau 1.5 s)
        CHECK(s.fk.phase() == raven::MissionPhase::LAND, "phase %s (mission dtg %.1f)",
              raven::missionPhaseName(s.fk.phase()), s.st.mission_distance_to_go);
        // Settled over the final point: the descent starts (only a commanded
        // descent arms the touchdown detector, so a disarm below proves it).
        s.run(2);
        CHECK(s.fk.armed(), "disarmed before touchdown");
        s.raw.baro_climb_rate = -0.8f;
        s.run(5);
        CHECK(s.fk.armed(), "disarmed while descending");
        s.raw.baro_climb_rate = 0.0f;
        s.run(12);
        CHECK(!s.fk.armed() && s.fk.phase() == raven::MissionPhase::NONE, "not disarmed after landing (armed=%d phase=%s)",
              s.fk.armed(), raven::missionPhaseName(s.fk.phase()));
        CHECK(!s.fk.arm(&why) && std::strstr(why, "already flown"), "re-arm after mission: %s", why);
    }
}

static void testTerrain() {
    std::printf("Terrain (DEM tiles)\n");
    // Pure index math
    int lat0, lon0, row, col;
    terrain::sampleIndex(39.9999, 32.0001, lat0, lon0, row, col);
    CHECK(lat0 == 39 && lon0 == 32 && row == 0 && col == 0, "NW corner sample %d %d %d %d", lat0, lon0, row, col);
    terrain::sampleIndex(39.0001, 32.9999, lat0, lon0, row, col);
    CHECK(row == 3599 && col == 3599, "SE corner sample %d %d", row, col);
    CHECK(terrain::byteOffset(1, 2) == (3600 + 2) * 2, "byte offset");

    const char *dir = "test/vtol_bin_tiles";
    FILE *probe = std::fopen("test/vtol_bin_tiles/N39E032.bin", "rb");
    if (!probe) { std::printf("    (tiles not present - real-data checks skipped)\n"); return; }
    std::fclose(probe);
    terrain::begin(0, dir);
    // Known landmarks: validates the tile naming, north-first row order and
    // georeferencing of the tar-to-bin.py output.
    const float tuz  = terrain::lookupElevation(38.75, 33.35);
    const float van  = terrain::lookupElevation(38.65, 42.90);
    const float sea  = terrain::lookupElevation(40.75, 28.20);
    const float erc  = terrain::lookupMaxElevation(38.5311, 35.4467, 4);
    const float ara  = terrain::lookupMaxElevation(39.7017, 44.2983, 4);
    CHECK(NEAR(tuz, 905, 10), "Lake Tuz %.0f", tuz);
    CHECK(NEAR(van, 1645, 15), "Lake Van %.0f", van);
    CHECK(sea == 0.0f, "Sea of Marmara %.0f", sea);
    CHECK(erc > 3850 && erc < 3950, "Erciyes %.0f (3917)", erc);
    CHECK(ara > 5050 && ara < 5160, "Ararat %.0f (5137)", ara);
    // N-S profile through Erciyes peaks at the summit (row order)
    float best = -1; int bestK = 99;
    for (int k = -4; k <= 4; ++k) {
        const float e = terrain::lookupElevation(38.5311 + k * 0.005, 35.4467);
        if (e > best) { best = e; bestK = k; }
    }
    CHECK(bestK == 0, "Erciyes N-S profile peaks at offset %d", bestK);
    // Footprint crossing a tile corner (4 tiles)
    const float corner = terrain::lookupMaxElevation(39.0, 33.0, 2);
    CHECK(!std::isnan(corner), "corner footprint unavailable");
    // Outside coverage -> NAN, never a low elevation
    CHECK(std::isnan(terrain::lookupElevation(35.5, 30.5)), "outside coverage not NAN");
    CHECK(std::isnan(terrain::lookupMaxElevation(36.00001, 30.5, 2)), "footprint half outside coverage not NAN");

    // Boot-time route check (terrain::checkRoute) on real routes: the tile set
    // ends at 36 N, so a route 10" (309 m) north of it is inside the DEM but
    // its 500 m drift corridor is not; 1' (1.85 km) north is fine.
    auto pathPoint = [](float s, double &lat, double &lon) -> bool {
        float x, y;
        if (!fcode::pathPointAt(s, x, y)) return false;
        fcode::localToGps(x, y, lat, lon);
        return true;
    };
    auto route = [&](const char *mission, const char *what, bool expectOk) {
        CHECK(fcode::begin(mission), "%s: parse %s", what, fcode::info().error);
        const terrain::RouteCheck rc = terrain::checkRoute(fcode::info().total_length_m, pathPoint,
                                                           60.0f, 500.0f, 1);
        CHECK(rc.ok == expectOk, "%s: ok=%d (missing %d/%d, first at %.0f m)", what, rc.ok,
              rc.missing, rc.points, rc.first_missing_m);
        std::printf("    route %-34s ok=%d points=%d tiles=%d max=%.0f m\n", what, rc.ok, rc.points,
                    rc.tiles, rc.max_elev_m);
    };
    route("F43; Z50;\nF90; X0325000; Y395000;\nF01; X+1200.0; Y+0.0;\nF01; X+0.0; Y+1200.0;\n"
          "F01; X-1200.0; Y+0.0;\nF01; X+0.0; Y-1200.0;\nF39;\n",
          "4-tile loop around 40N 33E", true);
    route("F43; Z50;\nF90; X0303000; Y360010;\nF01; X+600.0; Y+0.0;\nF39;\n",
          "309 m from the DEM edge", false);
    route("F43; Z50;\nF90; X0303000; Y360100;\nF01; X+600.0; Y+0.0;\nF39;\n",
          "1.85 km from the DEM edge", true);

    // A tile truncated while copying to the SD card is refused outright.
    namespace fs = std::filesystem;
    const fs::path tmp = fs::temp_directory_path() / "raven_truncated_tiles";
    fs::create_directories(tmp);
    { std::ofstream(tmp / "N39E032.bin", std::ios::binary) << std::string(4096, '\0'); }
    terrain::begin(0, tmp.string().c_str());
    CHECK(!terrain::tileAvailable(39.5, 32.5), "truncated tile accepted");
    CHECK(std::isnan(terrain::lookupElevation(39.0001, 32.0001)), "truncated tile read");
    fs::remove_all(tmp);
    terrain::begin(0, dir);
}

int main() {
    testNmea();
    testFcode();
    testTerrain();
    testFlightKinematics();
    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
