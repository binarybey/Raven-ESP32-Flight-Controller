#include "Arduino.h"
#include <SD.h>
#include "esp_timer.h"
#include "I2C_IO.h"
#include "IMU.h"
#include "barometer.h"
#include "readgnss.h"
#include "ahrs.h"
#include "air_data.h"
#include "terrain.h"
#include "terrain_following.h"
#include "fcode_interpreter.h"
#include "flight_kinematics.h"
#include "hardware_interface.h"
#include "command_console.h"
#include "writeTelemetry.h"

#define SD_CS_PIN    5
#define SD_MAX_FILES 8                   // log + up to 4 DEM tiles + mission + spare
#define FCODE_PATH   "/mission.fcode"
#define TERRAIN_DIR  "/vtol_bin_tiles"

// Task periods. The IMU task paces the control task: one control cycle per
// IMU sample, so the two can never drift apart.
#define IMU_PERIOD_MS       10     // 100 Hz = control rate
#define BARO_PERIOD_MS      500    // 2 Hz
#define GNSS_POLL_MS        50     // 20 Hz UART drain
#define GNSS_PRINT_MS       5000   // verbose GNSS block
#define AIRDATA_PERIOD_MS   20     // 50 Hz pitot
#define TERRAIN_PERIOD_MS   1000   // 1 Hz DEM lookups (SD reads - never in the control loop)
#define TELEMETRY_PERIOD_MS 100    // 10 Hz status line + SD log row
#define CONSOLE_POLL_MS     20

SemaphoreHandle_t gnssMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t baroMutex;
SemaphoreHandle_t airMutex;
SemaphoreHandle_t navMutex;
SemaphoreHandle_t terrainMutex;
SemaphoreHandle_t telemMutex;
SemaphoreHandle_t setpointMutex;
SemaphoreHandle_t sdMutex;
QueueHandle_t consoleReqQueue;
QueueHandle_t consoleReplyQueue;
TaskHandle_t controlTaskHandle = NULL;

// Hardware backends (lib/Hardware Interface) - replace with your ESC/servo and
// pitot drivers when they're wired, e.g. `ServoEscBackend actuators;`.
raven::NullActuatorBackend actuators;
raven::NullAirDataBackend  airData;

// Flight kinematics - config/gains are the defaults in flight_kinematics.h.
raven::VehicleConfig    fkConfig;
raven::ControlGains     fkGains;
raven::FlightKinematics fk;
raven::Setpoints        fkSetpoints;   // manual/RC fallback - nothing writes it yet

raven::TelemetryLogger logger;

// Data handed between tasks, each under its mutex above.
ImuSample         imuSample       = {};
BaroSnapshot      baroSnapshot    = {};
AirSnapshot       airSnapshot     = {};
NavSnapshot       navSnapshot     = {};
TerrainSnapshot   terrainSnapshot = {};
TelemetrySnapshot telemSnapshot;

bool pitotPresent = false;
bool terrainReady = false;
bool routeCovered = false;

void TaskGNSS(void *pvParameters) {
    uint32_t lastPrint = 0;
    for(;;) {
        if (readNMEA(&GNSS) && millis() - lastPrint >= GNSS_PRINT_MS) {
            printGNSS();
            lastPrint = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_MS));   // 20Hz loop
    }
}

void TaskIMU(void *pvParameters) {
    ImuSample sample = {};
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_PERIOD_MS));   // 100Hz loop, no drift

        bool imuOk = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            imuOk = readImuSample(sample, esp_timer_get_time());
            xSemaphoreGive(i2cMutex);
        }
        if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            imuSample = sample;
            xSemaphoreGive(imuMutex);
        }
        // Wakes TaskFlightControl. A failed read sends nothing: the control
        // task times out, marks the IMU stale, and fails safe.
        if (imuOk) xTaskNotifyGive(controlTaskHandle);
        //printIMU(sample.accel, sample.gyro, sample.mag);
    }
}

void TaskBMP(void *pvParameters) {
    // Climb rate from successive readings - owned here because only this
    // task knows when a sample is genuinely new (see RateEstimator).
    raven::RateEstimator climbRateEstimator(0.3f);
    int64_t t_prev = esp_timer_get_time();
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(BARO_PERIOD_MS));   // 2Hz loop

        bool bmpsuccess = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            bmpsuccess = readBMP280(BMP280calib, bmp_raw, bmp_out);
            xSemaphoreGive(i2cMutex);
        }
        if (!bmpsuccess) continue;

        const int64_t t_now = esp_timer_get_time();
        const float dt_baro = (float)(t_now - t_prev) * 1.0e-6f;
        t_prev = t_now;
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

void TaskAirData(void *pvParameters) {
    PitotFilter pitot(1000.0f / AIRDATA_PERIOD_MS);   // zeroes over the first 2 s
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(AIRDATA_PERIOD_MS));   // 50Hz loop

        float dp = 0.0f;
        bool got = false;
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            got = airData.read(dp);   // backend contract: called with i2cMutex held
            xSemaphoreGive(i2cMutex);
        }
        if (!got || !pitot.update(dp)) continue;   // no new sample, or still zeroing

        if (xSemaphoreTake(airMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            airSnapshot.zeroed    = true;
            airSnapshot.q_pa      = pitot.q();
            airSnapshot.sample_us = esp_timer_get_time();
            xSemaphoreGive(airMutex);
        }
    }
}

void TaskTerrain(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TERRAIN_PERIOD_MS));   // 1Hz loop

        NavSnapshot nav = {};
        if (xSemaphoreTake(navMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            nav = navSnapshot;
            xSemaphoreGive(navMutex);
        }
        const TerrainSnapshot terrain = lookupTerrain(nav, sdMutex);
        if (xSemaphoreTake(terrainMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            terrainSnapshot = terrain;
            xSemaphoreGive(terrainMutex);
        }
    }
}

void TaskFlightControl(void *pvParameters) {
    // Last good copy of every input: if a mutex is busy this cycle the old
    // copy is reused ("same data as last cycle") - never "no data".
    ImuSample        imu     = {};
    BaroSnapshot     baro    = {};
    GNSSData         gnss    = {};
    AirSnapshot      air     = {};
    TerrainSnapshot  terrain = {};
    raven::Setpoints manual;

    int64_t  t_prev = esp_timer_get_time();
    int64_t  rateWindowStart = t_prev;
    uint32_t cycles = 0;
    float    loopHz = 0.0f;

    for(;;) {
        // One cycle per IMU sample (100 Hz); the timeout means the IMU stalled.
        const bool gotImu = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(25)) > 0;

        // Real dt, not the nominal 10 ms - FlightKinematics clamps it.
        const int64_t t_now = esp_timer_get_time();
        const float dt = (float)(t_now - t_prev) * 1.0e-6f;
        t_prev = t_now;

        if (xSemaphoreTake(imuMutex,      pdMS_TO_TICKS(5)) == pdTRUE) { imu     = imuSample;       xSemaphoreGive(imuMutex); }
        if (xSemaphoreTake(baroMutex,     pdMS_TO_TICKS(2)) == pdTRUE) { baro    = baroSnapshot;    xSemaphoreGive(baroMutex); }
        if (xSemaphoreTake(gnssMutex,     pdMS_TO_TICKS(2)) == pdTRUE) { gnss    = masterGNSSData;  xSemaphoreGive(gnssMutex); }
        if (xSemaphoreTake(airMutex,      pdMS_TO_TICKS(2)) == pdTRUE) { air     = airSnapshot;     xSemaphoreGive(airMutex); }
        if (xSemaphoreTake(terrainMutex,  pdMS_TO_TICKS(2)) == pdTRUE) { terrain = terrainSnapshot; xSemaphoreGive(terrainMutex); }
        if (xSemaphoreTake(setpointMutex, pdMS_TO_TICKS(2)) == pdTRUE) { manual  = fkSetpoints;     xSemaphoreGive(setpointMutex); }

        updateAHRS(imu, gotImu, t_now, dt);

        raven::RawSensors raw;
        ahrsToRaw(raw);
        baroToRaw(baro, t_now, raw);
        gnssToRaw(gnss, raw);
        pitotToRaw(air, pitotPresent, t_now, raw);
        terrainToRaw(terrain, t_now, raw);

        // Operator commands - applied here because this task owns fk.
        raven::ConsoleRequest req;
        while (xQueueReceive(consoleReqQueue, &req, 0) == pdTRUE) {
            const raven::ConsoleReply rep = raven::executeCommand(fk, req.cmd);
            xQueueSend(consoleReplyQueue, &rep, 0);
        }

        raven::VehicleState st;
        const raven::ActuatorCmd out = fk.runCycle(dt, raw, manual, &st);
        actuators.write(raven::toActuatorOutputs(out));

        ++cycles;
        if (t_now - rateWindowStart >= 1000000) {
            loopHz = cycles * 1.0e6f / (float)(t_now - rateWindowStart);
            cycles = 0;
            rateWindowStart = t_now;
        }

        // No Serial in this task - hand the results to the terrain and telemetry tasks.
        if (xSemaphoreTake(navMutex, 0) == pdTRUE) {
            navSnapshot = makeNavSnapshot(st, fk.armed());
            xSemaphoreGive(navMutex);
        }
        if (xSemaphoreTake(telemMutex, 0) == pdTRUE) {
            telemSnapshot.st         = st;
            telemSnapshot.out        = out;
            telemSnapshot.raw        = raw;
            telemSnapshot.terrain_ms = terrain.lookup_ms;
            telemSnapshot.loop_hz    = loopHz;
            xSemaphoreGive(telemMutex);
        }
    }
}

void TaskTelemetry(void *pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    for(;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));   // 10Hz loop

        TelemetrySnapshot t;
        if (xSemaphoreTake(telemMutex, pdMS_TO_TICKS(5)) != pdTRUE) continue;
        t = telemSnapshot;
        xSemaphoreGive(telemMutex);

        printTelemetry(t);
        logger.logState(t.st, t.out);
    }
}

void TaskConsole(void *pvParameters) {
    raven::CommandConsole usb(Serial);
    // RF later: a transparent UART radio is just another Stream - give it a
    // second CommandConsole and handle it the same way.
    for(;;) {
        const raven::Command cmd = usb.poll();

        if (cmd == raven::Command::ARM || cmd == raven::Command::DISARM || cmd == raven::Command::LAND) {
            // fk belongs to TaskFlightControl: ask it, print its answer.
            const raven::ConsoleRequest req = { cmd };
            raven::ConsoleReply rep;
            xQueueReset(consoleReplyQueue);
            xQueueSend(consoleReqQueue, &req, 0);
            if (xQueueReceive(consoleReplyQueue, &rep, pdMS_TO_TICKS(300)) == pdTRUE)
                Serial.printf("%s: %s\n", rep.ok ? "OK" : "REFUSED", rep.msg);
            else
                Serial.println("ERROR: flight-control task did not answer");
        } else if (cmd == raven::Command::STATUS) {
            TelemetrySnapshot t;
            if (xSemaphoreTake(telemMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                t = telemSnapshot;
                xSemaphoreGive(telemMutex);
                raven::printStatus(Serial, t, terrainReady, routeCovered);
            }
        } else if (cmd == raven::Command::HELP) {
            Serial.print(raven::commandHelpText());
        } else if (cmd == raven::Command::UNKNOWN) {
            Serial.println("unknown command - type 'help'");
        }

        vTaskDelay(pdMS_TO_TICKS(CONSOLE_POLL_MS));
    }
}

void setup() {

    Serial.begin(115200);

    Wire.begin();
    Wire.setClock(400000);

    gnssMutex     = xSemaphoreCreateMutex();
    imuMutex      = xSemaphoreCreateMutex();
    i2cMutex      = xSemaphoreCreateMutex();
    baroMutex     = xSemaphoreCreateMutex();
    airMutex      = xSemaphoreCreateMutex();
    navMutex      = xSemaphoreCreateMutex();
    terrainMutex  = xSemaphoreCreateMutex();
    telemMutex    = xSemaphoreCreateMutex();
    setpointMutex = xSemaphoreCreateMutex();
    sdMutex       = xSemaphoreCreateMutex();
    consoleReqQueue   = xQueueCreate(4, sizeof(raven::ConsoleRequest));
    consoleReplyQueue = xQueueCreate(4, sizeof(raven::ConsoleReply));

    beginGNSS();

    readCalibrationData280(BMP280calib);
    printCalibration280(BMP280calib);
    initBMP280();

    setMPU(setMPU_DLPF(MPU_DLPF_2), setGyroFS(MPU_GYRO_FS_250), setAccFS(MPU_ACCEL_FS_2));
    setMag(magConfRegA(MAG_SAMPLE_AVG_8, MAG_OUTPUT_RATE_75), magConfRegB(MAG_SENS_1370), magModeReg(MAG_CONTINUOUS_MEAS));
    if (!measureGyroOffset(accel, gyro, gyroOffset))
        Serial.println("IMU NOT RESPONDING - gyro offset not measured; arming will be refused.");
    initAHRS(1000.0f / IMU_PERIOD_MS);

    // SD card: one mount for the mission, the DEM tiles and the log.
    if (!SD.begin(SD_CS_PIN, SPI, 4000000, "/sd", SD_MAX_FILES))
        Serial.println("SD card not mounted - no mission, terrain or log.");
    fcode::loadMission(FCODE_PATH, SD_CS_PIN);
    terrainReady = terrain::begin(SD_CS_PIN, TERRAIN_DIR);
    if (!terrainReady)
        Serial.println("Terrain: " TERRAIN_DIR " not found on SD - missions cannot be armed.");
    routeCovered = terrainReady && checkMissionRoute();
    if (logger.begin(sdMutex)) Serial.printf("Logging to SD: %s\n", logger.path());
    else                       Serial.println("SD logging unavailable.");

    const uint32_t configIssues = fk.begin(fkConfig, fkGains);
    printVehicleReport(fk, fkConfig, configIssues);
    fk.setMissionTerrainOk(routeCovered);

    if (!actuators.begin()) Serial.println("Actuator backend failed to start.");
    pitotPresent = airData.begin();
    Serial.println(pitotPresent ? "Pitot: present (zeroing for 2 s - keep it still/covered)"
                                : "Pitot: none - conversion to forward flight is disabled.");
    Serial.println("DISARMED. Type 'help' for commands.");

    // Control task first: TaskIMU notifies it through controlTaskHandle.
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
