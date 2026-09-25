#include "Arduino.h"
#include "readgnss.h"
#include "I2C_IO.h"
#include "barometer.h"
#include "IMU.h"
#include "Fusion.h"
#include "esp_timer.h"
#include "flight_kinematics.h"
#include "fcode_interpreter.h"
#include "terrain.h"
#include "hardware_interface.h"
#include "command_console.h"
#include "writeTelemetry.h"

#define SD_CS_PIN 5
#define FCODE_PATH "/mission.fcode"

// ---------------------------------------------------------------------------
// Task timing. The IMU task is the timing master: every fresh IMU sample
// wakes the flight-control task, so the two can never drift apart or
// consume the same sample twice.
// ---------------------------------------------------------------------------
static const uint32_t kImuPeriodMs       = 10;    // 100 Hz = control rate
static const uint32_t kBaroPeriodMs      = 500;   // 2 Hz
static const uint32_t kGnssPollMs        = 50;    // 20 Hz UART drain (latency, 1 KB buffer)
static const uint32_t kGnssPrintPeriodMs = 5000;  // verbose GNSS block
static const uint32_t kAirDataPeriodMs   = 20;    // 50 Hz pitot
static const uint32_t kTelemetryPeriodMs = 100;   // 10 Hz status line + SD log row
static const uint32_t kTerrainPeriodMs   = 1000;  // 1 Hz DEM lookups (SD seeks, core 0)

static const int64_t kImuFreshUs   = 30000;       // 3 IMU periods
static const int64_t kMagFreshUs   = 50000;       // HMC5883L runs at 75 Hz
static const int64_t kBaroFreshUs  = 1000000;     // 2 baro periods
static const int64_t kPitotFreshUs = 200000;      // see hardware_interface.h
static const int64_t kTerrainFreshUs = 3000000;   // 3 missed terrain updates

// Terrain look-ahead: the clearance is held above the highest DEM sample
// under the vehicle (5x5 samples, ~+/-60 m: GPS error + grid registration)
// and along the next stretch of the PLANNED path (3x3 every 60 m), so it
// climbs before rising ground instead of after it.
static const int   kTerrainUnderRadius  = 2;
static const int   kTerrainAheadRadius  = 1;
static const float kLookAheadTimeS      = 15.0f;   // look-ahead = ground speed * this,
static const float kLookAheadMinM       = 150.0f;  // clamped to [min, max]
static const float kLookAheadMaxM       = 600.0f;
static const float kLookAheadStepM      = 60.0f;
static const float kRouteCheckStepM     = 30.0f;   // boot-time DEM coverage check

FusionAhrs ahrs;
FusionBias bias;

SemaphoreHandle_t gnssMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t baroMutex;
SemaphoreHandle_t setpointMutex;
SemaphoreHandle_t airMutex;
SemaphoreHandle_t sdMutex;
SemaphoreHandle_t telemMutex;
SemaphoreHandle_t navMutex;
SemaphoreHandle_t terrainMutex;

// ---------------------------------------------------------------------------
// Hardware backends - THE plug-in point for the ESC/servo and pitot
// libraries (contract: lib/Hardware Interface/hardware_interface.h). Until
// they exist, the Null backends make outputs go nowhere and report "no
// pitot" (which keeps the aircraft in hover). To plug yours in:
//     #include "your_lib.h"
//     static YourActuators yourActuators;
//     raven::ActuatorBackend *actuators = &yourActuators;
// ---------------------------------------------------------------------------
static raven::NullActuatorBackend nullActuators;
static raven::NullAirDataBackend  nullAirData;
raven::ActuatorBackend *actuators = &nullActuators;
raven::AirDataBackend  *airData   = &nullAirData;
static bool pitotPresent = false;

// ---------------------------------------------------------------------------
// Flight kinematics. `fkConfig`/`fkGains` hold the struct defaults from
// flight_kinematics.h - mass/inertia/l_y are from the CAD, everything marked
// PLACEHOLDER there still needs real numbers (validateConfig() reports at
// boot).
//
// F-Code guidance, GNSS position handling, altitude fusion, the mission
// lifecycle and failsafes all live inside FlightKinematics - this file's
// jobs are: load the mission at boot, run the sensor tasks, gather raw
// sensor values into RawSensors, make one runCycle() call per IMU sample,
// and hand the result to the actuator backend.
// ---------------------------------------------------------------------------
raven::VehicleConfig    fkConfig;
raven::ControlGains     fkGains;
raven::FlightKinematics fk;

// Setpoints: manual/RC/telemetry fallback, used only when no F-Code mission
// is loaded (runCycle() decides that internally). Nothing writes this yet -
// it defaults to hold level / zero climb rate.
raven::Setpoints fkSetpoints;

// Latest IMU sample times, written by TaskIMU under imuMutex.
static int64_t imuSampleUs = 0;
static int64_t magSampleUs = 0;
static TaskHandle_t controlTaskHandle = nullptr;

// Barometer snapshot, shared between TaskBMP (writer) and TaskFlightControl
// (reader). Separate from i2cMutex on purpose - that one protects the I2C
// bus itself, not this computed result.
struct BaroSnapshot {
    bool    valid         = false;
    float   altitude      = 0.0f;   // m
    float   climb_rate    = 0.0f;   // m/s, positive up
    float   pressure_pa   = 101325.0f;
    float   temperature_c = 15.0f;
    int64_t sample_us     = 0;
};
BaroSnapshot baroSnapshot;

// Pitot snapshot (TaskAirData -> TaskFlightControl).
struct AirSnapshot {
    bool    zeroed    = false;
    float   q_pa      = 0.0f;       // zero-offset removed, low-passed
    int64_t sample_us = 0;
};
AirSnapshot airSnapshot;

// Position for the terrain lookups (TaskFlightControl -> TaskTerrain).
struct NavSnapshot {
    bool   valid        = false;
    double lat = 0.0, lon = 0.0;    // dead-reckoned estimate, not the 1 Hz raw fix
    bool   mission      = false;
    float  path_s       = 0.0f;     // m flown along the planned path
    float  ground_speed = 0.0f;
};
static NavSnapshot navSnapshot;

// Terrain result (TaskTerrain -> TaskFlightControl).
struct TerrainSnapshot {
    bool     valid     = false;
    float    elev_m    = 0.0f;      // max under + look-ahead
    float    under_m   = 0.0f;      // max under the vehicle only
    uint32_t lookup_ms = 0;
    int64_t  sample_us = 0;
};
static TerrainSnapshot terrainSnapshot;
static bool terrainReady   = false;   // DEM directory found at boot
static bool routeTerrainOk = false;   // boot check: DEM covers the whole mission route

// Telemetry snapshot (TaskFlightControl -> TaskTelemetry / console).
struct TelemetrySnapshot {
    raven::VehicleState st;
    raven::ActuatorCmd  out;
    bool     imu_ok = false, mag_ok = false, ahrs_ready = false, baro_ok = false, pitot_ok = false;
    bool     terrain_ok = false;
    uint32_t terrain_ms = 0;
    uint8_t  gnss_fix = 0, gnss_sats = 0;
    float    gnss_hdop = 99.9f;
    float    loop_hz = 0.0f;
};
static TelemetrySnapshot telemSnapshot;

// Operator commands: console task -> control task (which owns `fk`) and back.
struct ConsoleRequest { raven::Command cmd; };
struct ConsoleReply   { bool ok; const char *msg; };
static QueueHandle_t consoleReqQueue;
static QueueHandle_t consoleReplyQueue;

static raven::TelemetryLogger logger;

// ===========================================================================
// Sensor tasks (core 0)
// ===========================================================================
void TaskGNSS(void *pvParameters) {
    uint32_t lastPrint = 0;
    for(;;) {
        if (readNMEA(&GNSS) && millis() - lastPrint >= kGnssPrintPeriodMs) {
            printGNSS();
            lastPrint = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(kGnssPollMs));
    }
}

void TaskIMU(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kImuPeriodMs));   // fixed 100 Hz, no drift

        bool imuOk = false, magOk = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            imuOk = readMPU(accel, gyro);
            magOk = readMag(mag);
            xSemaphoreGive(i2cMutex);
        }
        const int64_t now = esp_timer_get_time();
        if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            if (imuOk) {
                applyAccelCalibration(accel, accelC);
                calibrateGyro(gyro, gyroOffset, gyroC);
                imuSampleUs = now;
            }
            if (magOk) {
                applyMagCalibration(mag, magC);
                magSampleUs = now;
            }
            xSemaphoreGive(imuMutex);
        }
        // A failed read sends no notification: the control task then times
        // out, marks the IMU stale, and FlightKinematics fails safe.
        if (imuOk && controlTaskHandle != nullptr) xTaskNotifyGive(controlTaskHandle);
        //printIMU(accelC, gyro, magC);
        //printUncalibratedMag(mag);
        //printUncalibratedAccel(accel);
    }
}

void TaskBMP(void *pvParameters) {
    // Cutoff well under the ~2Hz sample rate this task runs at. Owned here,
    // not in flight_kinematics, because only this task knows when a sample
    // is genuinely new - see RateEstimator's header comment.
    static raven::RateEstimator climbRateEstimator(0.3f);
    int64_t t_prev_baro = esp_timer_get_time();
    TickType_t lastWake = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kBaroPeriodMs));   // 2Hz loop
        bool bmpsuccess = false;

        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            bmpsuccess = readBMP280(BMP280calib, bmp_raw, bmp_out);
            xSemaphoreGive(i2cMutex);
        }

        if (bmpsuccess) {
            const int64_t t_now = esp_timer_get_time();
            const float dt_baro = (float)(t_now - t_prev_baro) * 1.0e-6f;
            t_prev_baro = t_now;

            const float climb_rate = climbRateEstimator.update((float)bmp_out.altitude, dt_baro);

            if (xSemaphoreTake(baroMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                baroSnapshot.valid         = true;
                baroSnapshot.altitude      = (float)bmp_out.altitude;
                baroSnapshot.climb_rate    = climb_rate;
                baroSnapshot.pressure_pa   = (float)bmp_out.pressure;
                baroSnapshot.temperature_c = (float)bmp_out.temperature;
                baroSnapshot.sample_us     = t_now;
                xSemaphoreGive(baroMutex);
            }
            //printPressTemp(bmp_out);
        }
    }
}

// Pitot: poll the backend, auto-zero over the first 2 s, low-pass, publish.
// Only started if airData->begin() found a sensor.
void TaskAirData(void *pvParameters) {
    static const int   kZeroSamples = 100;                  // 2 s at 50 Hz
    static const float kDt          = kAirDataPeriodMs * 1.0e-3f;
    static const float kCutoffHz    = 5.0f;
    const float lpAlpha = kDt / (1.0f / (2.0f * 3.14159265f * kCutoffHz) + kDt);

    double zeroSum = 0.0;
    int    zeroCount = 0;
    float  offset = 0.0f, filtered = 0.0f;
    bool   zeroed = false;
    TickType_t lastWake = xTaskGetTickCount();

    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kAirDataPeriodMs));
        float dp = 0.0f;
        bool got = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            got = airData->read(dp);       // contract: called with i2cMutex held
            xSemaphoreGive(i2cMutex);
        }
        if (!got) continue;

        if (!zeroed) {
            zeroSum += dp;
            if (++zeroCount >= kZeroSamples) {
                offset   = (float)(zeroSum / zeroCount);
                filtered = 0.0f;
                zeroed   = true;
                Serial.printf("Pitot zeroed: offset %.1f Pa\n", offset);
            }
            continue;
        }
        filtered += lpAlpha * ((dp - offset) - filtered);

        if (xSemaphoreTake(airMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            airSnapshot.zeroed    = true;
            airSnapshot.q_pa      = filtered;
            airSnapshot.sample_us = esp_timer_get_time();
            xSemaphoreGive(airMutex);
        }
    }
}

// Terrain: DEM lookups under the vehicle and along the path ahead. SD reads
// take milliseconds, so this runs here at 1 Hz, never in the control loop.
static float terrainLookupLocked(double lat, double lon, int radius) {
    float e = NAN;
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {   // SD shared with the logger
        e = terrain::lookupMaxElevation(lat, lon, radius);
        xSemaphoreGive(sdMutex);
    }
    return e;
}

void TaskTerrain(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kTerrainPeriodMs));
        NavSnapshot nav;
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            nav = navSnapshot;
            xSemaphoreGive(navMutex);
        }
        TerrainSnapshot out;
        if (nav.valid) {
            const uint32_t t0 = millis();
            const float under = terrainLookupLocked(nav.lat, nav.lon, kTerrainUnderRadius);
            bool  ok   = !isnan(under);
            float best = under;
            if (ok && nav.mission) {
                const float total = fcode::info().total_length_m;
                float ahead = nav.ground_speed * kLookAheadTimeS;
                if (ahead < kLookAheadMinM) ahead = kLookAheadMinM;
                if (ahead > kLookAheadMaxM) ahead = kLookAheadMaxM;
                for (float d = kLookAheadStepM; d <= ahead + 1.0f && ok; d += kLookAheadStepM) {
                    const float s_ahead = fminf(nav.path_s + d, total);
                    float x, y;
                    double lat, lon;
                    fcode::pathPointAt(s_ahead, x, y);
                    fcode::localToGps(x, y, lat, lon);
                    const float e = terrainLookupLocked(lat, lon, kTerrainAheadRadius);
                    if (isnan(e)) ok = false;
                    else if (e > best) best = e;
                    if (s_ahead >= total) break;
                }
            }
            out.valid     = ok;
            out.elev_m    = best;
            out.under_m   = under;
            out.lookup_ms = millis() - t0;
            out.sample_us = esp_timer_get_time();
        }
        if (xSemaphoreTake(terrainMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            terrainSnapshot = out;
            xSemaphoreGive(terrainMutex);
        }
    }
}

// ===========================================================================
// Flight control (core 1, highest priority). No Serial I/O in here - it
// would block the loop whenever the UART buffer is full.
// ===========================================================================
void TaskFlightControl(void *pvParameters) {

    int64_t t_prev = esp_timer_get_time();
    FusionVector lastGyro = FUSION_VECTOR_ZERO;
    GNSSData gnssCache = {};         // last good copy - reused if the mutex is busy
    BaroSnapshot baroCache;
    AirSnapshot  airCache;
    TerrainSnapshot terrainCache;
    raven::Setpoints spCache;
    uint32_t cycles = 0;
    int64_t  rateWindowStart = t_prev;
    float    loopHz = 0.0f;

    for(;;) {
        // Wait for the next IMU sample (100 Hz). Timeout = IMU stalled.
        const bool gotImu = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25)) > 0;

        // Real dt, not the nominal 10ms - jitter and scheduling delay show up
        // here, and the PID D-terms and integrators need the actual elapsed
        // time. flight_kinematics clamps it internally.
        const int64_t t_now = esp_timer_get_time();
        const float dt = (float)(t_now - t_prev) * 1.0e-6f;
        t_prev = t_now;

        // ---- IMU snapshot ----
        FusionVector gyroscope = FUSION_VECTOR_ZERO, accelerometer = FUSION_VECTOR_ZERO,
                     magnetometer = FUSION_VECTOR_ZERO;
        int64_t imuUs = 0, magUs = 0;
        if (xSemaphoreTake(imuMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            gyroscope     = { .axis = {gyroC.x, gyroC.y, gyroC.z} };
            accelerometer = { .axis = {accelC.x, accelC.y, accelC.z} };
            magnetometer  = { .axis = {magC.x, magC.y, magC.z} };
            imuUs = imuSampleUs;
            magUs = magSampleUs;
            xSemaphoreGive(imuMutex);
        }
        const bool imuFresh = gotImu && (t_now - imuUs) < kImuFreshUs;
        const bool magFresh = (t_now - magUs) < kMagFreshUs;

        if (imuFresh) {
            gyroscope = FusionBiasUpdate(&bias, gyroscope);
            // Fusion otherwise assumes exactly settings.sampleRate; use the
            // measured period (bounded) for the gyro integration.
            float period = dt;
            if (period < 0.002f) period = 0.002f;
            if (period > 0.05f)  period = 0.05f;
            FusionAhrsSetSamplePeriod(&ahrs, period);
            if (magFresh) FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);
            else          FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);
            lastGyro = gyroscope;
        }
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // --- gather raw sensors as plain floats. Everything downstream
        // (NWU/declination correction, guidance, altitude fusion) happens
        // inside flight_kinematics.cpp's runCycle().
        raven::RawSensors raw;
        raw.imu_valid         = imuFresh;
        raw.mag_valid         = magFresh;
        raw.ahrs_ready        = !FusionAhrsGetFlags(&ahrs).startup;
        raw.fusion_roll_deg   = euler.angle.roll;
        raw.fusion_pitch_deg  = euler.angle.pitch;
        raw.fusion_yaw_deg    = euler.angle.yaw;
        raw.fusion_gyro_x_dps = lastGyro.axis.x;
        raw.fusion_gyro_y_dps = lastGyro.axis.y;
        raw.fusion_gyro_z_dps = lastGyro.axis.z;

        // ---- barometer ----
        if (xSemaphoreTake(baroMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            baroCache = baroSnapshot;
            xSemaphoreGive(baroMutex);
        }
        raw.baro_valid         = baroCache.valid && (t_now - baroCache.sample_us) < kBaroFreshUs;
        raw.baro_altitude      = baroCache.altitude;
        raw.baro_climb_rate    = baroCache.climb_rate;
        raw.baro_pressure_pa   = baroCache.pressure_pa;
        raw.baro_temperature_c = baroCache.temperature_c;

        // ---- GNSS ---- (a busy mutex means "same data as last cycle", which
        // the sequence counters make harmless - never "no fix, speed 0")
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            gnssCache = masterGNSSData;
            xSemaphoreGive(gnssMutex);
        }
        raw.gnss_fix_valid = (gnssCache.fixQuality > 0);
        if (raw.gnss_fix_valid) {
            raw.gnss_lat_deg    = gnssCache.latitude_deg;
            raw.gnss_lon_deg    = gnssCache.longitude_deg;
            raw.gnss_altitude_m = gnssCache.true_Altitude;
        }
        raw.gnss_hdop    = gnssCache.HDOP;
        raw.gnss_sats    = gnssCache.satellite_number_active;
        raw.gnss_fix_seq = gnssCache.fixSeq;
        // RMC speed is in knots; flight_kinematics expects m/s. No course
        // (typical when stationary) is treated as not moving.
        raw.gnss_vel_valid    = gnssCache.rmcValid;
        raw.gnss_ground_speed = gnssCache.courseValid ? gnssCache.ground_Speed * 0.514444f : 0.0f;
        raw.gnss_course_rad   = gnssCache.courseValid ? gnssCache.course_Degrees * DEG_TO_RAD : 0.0f;
        raw.gnss_vel_seq      = gnssCache.velSeq;

        // ---- pitot ----
        if (pitotPresent && xSemaphoreTake(airMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            airCache = airSnapshot;
            xSemaphoreGive(airMutex);
        }
        raw.pitot_valid = pitotPresent && airCache.zeroed &&
                          (t_now - airCache.sample_us) < kPitotFreshUs &&
                          airCache.q_pa > -50.0f && airCache.q_pa < 5000.0f;
        raw.pitot_q_pa  = airCache.q_pa;

        // ---- terrain (DEM) ----
        if (xSemaphoreTake(terrainMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            terrainCache = terrainSnapshot;
            xSemaphoreGive(terrainMutex);
        }
        raw.terrain_valid  = terrainCache.valid && (t_now - terrainCache.sample_us) < kTerrainFreshUs;
        raw.terrain_elev_m = terrainCache.elev_m;

        // ---- manual setpoints (RC/telemetry, when one exists) ----
        if (xSemaphoreTake(setpointMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            spCache = fkSetpoints;
            xSemaphoreGive(setpointMutex);
        }

        // ---- operator commands (this task owns `fk`, so it applies them) ----
        ConsoleRequest req;
        while (xQueueReceive(consoleReqQueue, &req, 0) == pdTRUE) {
            ConsoleReply rep = { false, "unsupported" };
            switch (req.cmd) {
                case raven::Command::ARM:
                    rep.ok = fk.arm(&rep.msg);
                    break;
                case raven::Command::DISARM:
                    fk.disarm();
                    rep = { true, "disarmed - motors off" };
                    break;
                case raven::Command::LAND:
                    if (!fk.armed()) {
                        rep = { false, "not armed" };
                    } else {
                        fk.requestFailsafe(raven::FailsafeReason::COMMANDED);
                        rep = { true, "landing at current position" };
                    }
                    break;
                default:
                    break;
            }
            xQueueSend(consoleReplyQueue, &rep, 0);
        }

        raven::VehicleState st;
        const raven::ActuatorCmd out = fk.runCycle(dt, raw, spCache, &st);

        // ---- position for the terrain task ----
        if (xSemaphoreTake(navMutex, 0) == pdTRUE) {
            navSnapshot.valid = st.nav_valid;
            if (st.nav_valid) fcode::localToGps(st.pos_x, st.pos_y, navSnapshot.lat, navSnapshot.lon);
            navSnapshot.mission      = fcode::loaded();
            navSnapshot.ground_speed = st.ground_speed;
            // Before arming, look ahead from the start of the route (the
            // takeoff climb must clear the first leg); afterwards from where
            // the mission says we are.
            float path_s = 0.0f;
            if (fcode::loaded() && (fk.armed() || fcode::complete())) {
                const float total = fcode::info().total_length_m;
                path_s = total - st.mission_distance_to_go;
                if (path_s < 0.0f)  path_s = 0.0f;
                if (path_s > total) path_s = total;
            }
            navSnapshot.path_s = path_s;
            xSemaphoreGive(navMutex);
        }

        // ---- outputs ----
        raven::ActuatorOutputs hw;
        hw.armed             = out.armed;
        hw.throttle_left     = out.armed ? out.thrust_left  : 0.0f;
        hw.throttle_right    = out.armed ? out.thrust_right : 0.0f;
        hw.nacelle_left_rad  = out.nacelle_left;
        hw.nacelle_right_rad = out.nacelle_right;
        hw.aileron           = out.aileron;
        hw.elevator          = out.elevator;
        hw.rudder            = out.rudder;
        actuators->write(hw);

        // ---- loop rate + telemetry snapshot ----
        ++cycles;
        if (t_now - rateWindowStart >= 1000000) {
            loopHz = cycles * 1.0e6f / (float)(t_now - rateWindowStart);
            cycles = 0;
            rateWindowStart = t_now;
        }
        if (xSemaphoreTake(telemMutex, 0) == pdTRUE) {
            telemSnapshot.st         = st;
            telemSnapshot.out        = out;
            telemSnapshot.imu_ok     = raw.imu_valid;
            telemSnapshot.mag_ok     = raw.mag_valid;
            telemSnapshot.ahrs_ready = raw.ahrs_ready;
            telemSnapshot.baro_ok    = raw.baro_valid;
            telemSnapshot.pitot_ok   = raw.pitot_valid;
            telemSnapshot.terrain_ok = raw.terrain_valid;
            telemSnapshot.terrain_ms = terrainCache.lookup_ms;
            telemSnapshot.gnss_fix   = gnssCache.fixQuality;
            telemSnapshot.gnss_sats  = gnssCache.satellite_number_active;
            telemSnapshot.gnss_hdop  = gnssCache.HDOP;
            telemSnapshot.loop_hz    = loopHz;
            xSemaphoreGive(telemMutex);
        }
    }
}

// ===========================================================================
// Telemetry + console (core 0, low priority)
// ===========================================================================
static bool readTelemetry(TelemetrySnapshot &s) {
    if (xSemaphoreTake(telemMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
    s = telemSnapshot;
    xSemaphoreGive(telemMutex);
    return true;
}

void TaskTelemetry(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    TelemetrySnapshot s;
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kTelemetryPeriodMs));
        if (!readTelemetry(s)) continue;
        const raven::ActuatorCmd  &o = s.out;
        const raven::VehicleState &v = s.st;

        // ~200 bytes at 10 Hz = 2 kB/s, well inside 115200 baud's 11.5 kB/s.
        // This is also the natural place for an RF telemetry downlink.
        Serial.printf("%-10s %-7s %s fs=%-9s r=%6.1f p=%6.1f y=%6.1f agl=%6.1f vz=%+5.1f as=%4.1f a=%5.1f "
                      "thr=%.2f/%.2f nac=%5.1f/%5.1f srf=%+.2f/%+.2f/%+.2f xte=%+6.1f dtg=%7.0f gs=%4.1f "
                      "ter=%5.0f nav=%d mag=%d pit=%d %3.0fHz\n",
            raven::flightModeName(o.mode), raven::missionPhaseName(o.phase),
            o.armed ? "ARMED" : "safe ", raven::failsafeReasonName(o.fs_reason),
            v.roll * 57.2958f, v.pitch * 57.2958f, v.yaw * 57.2958f,
            v.agl, v.climb_rate, v.airspeed, o.alpha_cmd * 57.2958f,
            o.thrust_left, o.thrust_right, o.nacelle_left * 57.2958f, o.nacelle_right * 57.2958f,
            o.aileron, o.elevator, o.rudder,
            o.cross_track_dbg, v.mission_distance_to_go, v.ground_speed,
            v.terrain_valid ? v.terrain_elev_m : -1.0f,
            v.nav_valid ? 1 : 0, s.mag_ok ? 1 : 0, s.pitot_ok ? 1 : 0, s.loop_hz);

        logger.logState(v, o);
    }
}

static void handleConsole(raven::CommandConsole &console) {
    const raven::Command cmd = console.poll();
    Stream &io = console.io();
    switch (cmd) {
        case raven::Command::NONE:
            return;

        case raven::Command::ARM:
        case raven::Command::DISARM:
        case raven::Command::LAND: {
            xQueueReset(consoleReplyQueue);
            ConsoleRequest req = { cmd };
            xQueueSend(consoleReqQueue, &req, 0);
            ConsoleReply rep;
            if (xQueueReceive(consoleReplyQueue, &rep, pdMS_TO_TICKS(300)) == pdTRUE) {
                io.printf("%s: %s\n", rep.ok ? "OK" : "REFUSED", rep.msg);
            } else {
                io.println("ERROR: flight-control task did not answer");
            }
            return;
        }

        case raven::Command::STATUS: {
            TelemetrySnapshot s;
            if (!readTelemetry(s)) { io.println("status unavailable"); return; }
            const fcode::MissionInfo &mi = fcode::info();
            io.printf("armed=%d mode=%s phase=%s failsafe=%s loop=%.0fHz\n",
                s.out.armed ? 1 : 0, raven::flightModeName(s.out.mode),
                raven::missionPhaseName(s.out.phase), raven::failsafeReasonName(s.out.fs_reason),
                s.loop_hz);
            io.printf("imu=%d mag=%d ahrs_ready=%d baro=%d pitot=%d gnss_fix=%d sats=%d hdop=%.1f nav=%d\n",
                s.imu_ok, s.mag_ok, s.ahrs_ready, s.baro_ok, s.pitot_ok,
                s.gnss_fix, s.gnss_sats, s.gnss_hdop, s.st.nav_valid ? 1 : 0);
            io.printf("mission: %s, %d segments, %.0f m total, %.0f m to go, clearance %.0f m\n",
                mi.ok ? "loaded" : "none", mi.segment_count, mi.total_length_m,
                s.st.mission_distance_to_go, mi.ground_clearance_m);
            io.printf("attitude r=%.1f p=%.1f y(true)=%.1f  agl=%.1f alt=%.1f\n",
                s.st.roll * 57.2958f, s.st.pitch * 57.2958f, s.st.yaw * 57.2958f,
                s.st.agl, s.st.altitude);
            io.printf("terrain: dem=%s route=%s now=%d elev=%.0f m lookup=%lu ms\n",
                terrainReady ? "ok" : "MISSING", routeTerrainOk ? "covered" : "NOT covered",
                s.terrain_ok ? 1 : 0, s.st.terrain_elev_m, (unsigned long)s.terrain_ms);
            return;
        }

        case raven::Command::HELP:
            io.print(raven::commandHelpText());
            return;

        case raven::Command::UNKNOWN:
            io.println("unknown command - type 'help'");
            return;
    }
}

void TaskConsole(void *pvParameters) {
    static raven::CommandConsole usbConsole(Serial);
    // RF later: a transparent UART radio is just another Stream, e.g.
    //   static raven::CommandConsole radioConsole(Serial1);   // after Serial1.begin(57600, SERIAL_8N1, RX, TX)
    // and call handleConsole(radioConsole) below as well.
    for(;;) {
        handleConsole(usbConsole);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ===========================================================================
// setup
// ===========================================================================
static void printMissionInfo() {
    const fcode::MissionInfo &mi = fcode::info();
    if (!mi.ok) {
        Serial.printf("F-Code parse FAILED: %s", mi.error ? mi.error : "?");
        if (mi.error_line > 0) Serial.printf(" (line %d)", mi.error_line);
        Serial.println(" - no mission, manual mode only.");
        return;
    }
    Serial.printf("F-Code mission: %d segments, %.0f m, clearance %.0f m, origin %.6f N %.6f E\n",
        mi.segment_count, mi.total_length_m, mi.ground_clearance_m,
        mi.origin_lat_deg, mi.origin_lon_deg);
    if (mi.unknown_lines > 0)
        Serial.printf("  note: %d line(s) with unknown opcodes ignored\n", mi.unknown_lines);
    if (mi.arc_warnings > 0)
        Serial.printf("  WARNING: %d arc(s) disagree with their R/A fields (worst %.1f%% radius, %.1f deg sweep)\n",
            mi.arc_warnings, mi.worst_arc_radius_err_pct, mi.worst_arc_sweep_err_deg);
}

void setup() {

    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    gnssMutex     = xSemaphoreCreateMutex();
    imuMutex      = xSemaphoreCreateMutex();
    i2cMutex      = xSemaphoreCreateMutex();
    baroMutex     = xSemaphoreCreateMutex();
    setpointMutex = xSemaphoreCreateMutex();
    airMutex      = xSemaphoreCreateMutex();
    sdMutex       = xSemaphoreCreateMutex();
    telemMutex    = xSemaphoreCreateMutex();
    navMutex      = xSemaphoreCreateMutex();
    terrainMutex  = xSemaphoreCreateMutex();
    consoleReqQueue   = xQueueCreate(4, sizeof(ConsoleRequest));
    consoleReplyQueue = xQueueCreate(4, sizeof(ConsoleReply));

    beginGNSS();

    readCalibrationData280(BMP280calib);
    printCalibration280(BMP280calib);
    initBMP280();

    setMPU(setMPU_DLPF(MPU_DLPF_2), setGyroFS(MPU_GYRO_FS_250), setAccFS(MPU_ACCEL_FS_2));
    setMag(magConfRegA(MAG_SAMPLE_AVG_8, MAG_OUTPUT_RATE_75), magConfRegB(MAG_SENS_1370), magModeReg(MAG_CONTINUOUS_MEAS));
    if (!measureGyroOffset(accel, gyro, gyroOffset)) {
        Serial.println("IMU NOT RESPONDING - gyro offset not measured; arming will be refused.");
    }

    FusionBiasInitialise(&bias);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .sampleRate = 100.0f,                  // = 1000 / kImuPeriodMs
        .convention = FusionConventionNwu,
        .gain = 0.5f,   // Default filter gain
        .gyroscopeRange = 250.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .rejectionTimeout = 5.0f,
    };

    FusionAhrsSetSettings(&ahrs, &settings);

    // F-Code mission: load from SD, then parse. Either step failing leaves
    // no mission loaded - the vehicle then only flies manual setpoints -
    // never a half-loaded or truncated mission.
    const char *fcodeText = fcode::loadFCodeFromSd(FCODE_PATH, SD_CS_PIN);
    if (fcodeText == nullptr) {
        Serial.printf("F-Code: %s - no mission, manual mode only.\n", fcode::loadError());
    } else {
        fcode::begin(fcodeText);
        printMissionInfo();
    }

    // Terrain (DEM) on the same SD card - the F43 clearance is held above it.
    terrainReady = terrain::begin(SD_CS_PIN);
    if (!terrainReady) {
        Serial.println("Terrain: /vtol_bin_tiles not found on SD - missions cannot be armed.");
    }

    if (logger.begin(sdMutex)) Serial.printf("Logging to SD: %s\n", logger.path());
    else                       Serial.println("SD logging unavailable.");

    const uint32_t issues = fk.begin(fkConfig, fkGains);
    Serial.printf("Vehicle: %.3f kg, hover thrust %.0f%% of max, wing stall %.1f m/s, cruise %.1f m/s\n",
        fkConfig.mass,
        100.0f * fkConfig.mass * fkConfig.g / (2.0f * fkConfig.thrust_max_per_rotor),
        fk.stallSpeed(1.225f), fkConfig.cruise_airspeed);
    for (uint32_t bit = 1; bit != 0 && bit <= issues; bit <<= 1) {
        if (issues & bit)
            Serial.printf("  CONFIG %s: %s\n", (bit & raven::CFG_FATAL_MASK) ? "FATAL" : "warning",
                raven::configIssueText(bit));
    }

    // Walk the whole planned route through the DEM before anything flies: a
    // missing tile mid-route must be found on the bench, not in the air.
    if (fcode::loaded() && terrainReady) {
        const uint32_t t0 = millis();
        const float total = fcode::info().total_length_m;
        int points = 0, missing = 0;
        float maxElev = -1.0e9f, firstMissingAt = -1.0f;
        for (float s = 0.0f; ; s += kRouteCheckStepM) {
            const float ss = fminf(s, total);
            float x, y;
            double lat, lon;
            fcode::pathPointAt(ss, x, y);
            fcode::localToGps(x, y, lat, lon);
            const float e = terrain::lookupMaxElevation(lat, lon, kTerrainAheadRadius);
            ++points;
            if (isnan(e)) { if (missing++ == 0) firstMissingAt = ss; }
            else if (e > maxElev) maxElev = e;
            if (ss >= total) break;
        }
        routeTerrainOk = (missing == 0);
        if (routeTerrainOk) {
            Serial.printf("Terrain route check: OK, %d points, highest terrain %.0f m -> up to %.0f m MSL (%lu ms)\n",
                points, maxElev, maxElev + fcode::info().ground_clearance_m, (unsigned long)(millis() - t0));
        } else {
            Serial.printf("Terrain route check: FAILED, %d of %d points without DEM (first at %.0f m along the route) - mission cannot be armed\n",
                missing, points, firstMissingAt);
        }
    }
    fk.setMissionTerrainOk(routeTerrainOk);

    if (!actuators->begin()) Serial.println("Actuator backend failed to start.");
    pitotPresent = airData->begin();
    Serial.println(pitotPresent ? "Pitot: present (zeroing for 2 s - keep it still/covered)"
                                : "Pitot: none - conversion to forward flight is disabled.");

    // Boot DISARMED. Arm from the console ('arm') once pre-arm checks pass.
    Serial.println("DISARMED. Type 'help' for commands.");

    // Control task first so its handle exists before the IMU starts
    // notifying it.
    xTaskCreatePinnedToCore(TaskFlightControl, "Control",   8192, NULL, 4, &controlTaskHandle, 1);
    xTaskCreatePinnedToCore(TaskIMU,           "IMU",       4096, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(TaskBMP,           "Barometer", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(TaskGNSS,          "GNSS",      4096, NULL, 2, NULL, 0);
    if (pitotPresent)
        xTaskCreatePinnedToCore(TaskAirData,   "AirData",   4096, NULL, 2, NULL, 0);
    if (terrainReady)
        xTaskCreatePinnedToCore(TaskTerrain,   "Terrain",   4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskTelemetry,     "Telemetry", 6144, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(TaskConsole,       "Console",   4096, NULL, 1, NULL, 0);

    vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL);
}
