#include "WiFi.h"
#include "Arduino.h"
#include "readgnss.h"
#include "I2C_IO.h"
#include "barometer.h"
#include "IMU.h"
#include "Fusion.h"
#include "esp_timer.h"
#include "flight_kinematics.h"

FusionAhrs ahrs;
FusionBias bias;

extern HardwareSerial GNSS;

SemaphoreHandle_t gnssMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t baroMutex;
SemaphoreHandle_t setpointMutex;

// ---------------------------------------------------------------------------
// Flight kinematics. `fkConfig`/`fkGains` below are the struct DEFAULTS from
// flight_kinematics.h - placeholders, not your airframe. Before first flight,
// at minimum set: mass, Ixx/Iyy/Izz (Fusion 360, about the CG), l_x/l_y/l_z
// (rotor geometry), thrust_max_per_rotor (static test), q_ref (~0.5*rho*Vcruise^2).
// ---------------------------------------------------------------------------
raven::VehicleConfig    fkConfig;
raven::ControlGains     fkGains;
raven::FlightKinematics fk;

// Setpoints: no F-Code parser wired in yet. Whatever task ends up loading and
// stepping through F-Code should write into this struct (under
// setpointMutex), the same way an RC task would. Until then this defaults to
// "hold position, zero rates".
raven::Setpoints fkSetpoints;

// Barometer snapshot, shared between TaskBMP (writer) and TaskFlightControl
// (reader). Separate from i2cMutex on purpose - that one protects the I2C
// bus itself, not this computed result.
struct BaroSnapshot {
    bool  valid         = false;
    float altitude      = 0.0f;   // m
    float climb_rate    = 0.0f;   // m/s, positive up
    float pressure_pa   = 101325.0f;
    float temperature_c = 15.0f;
};
BaroSnapshot baroSnapshot;

void TaskGNSS(void *pvParameters) {
  GNSS.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
    for(;;) {
        if(readNMEA(&GNSS)){
        printGNSS();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1Hz loop
    }
}

void TaskIMU(void *pvParameters) {
    for(;;) {

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            readMPU(accel, gyro);
            readMag(mag);
            xSemaphoreGive(i2cMutex);
        }
        if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            applyAccelCalibration(accel, accelC);
            applyMagCalibration(mag, magC);
            calibrateGyro(gyro, gyroOffset, gyroC);
            xSemaphoreGive(imuMutex);
        }
        //printIMU(accelC, gyro, magC);
        //printUncalibratedMag(mag);
        //printUncalibratedAccel(accel);
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz loop
    }
}

void TaskBMP(void *pvParameters) {
    // Cutoff well under the ~2Hz sample rate this task runs at. Owned here,
    // not in flight_kinematics, because only this task knows when a sample
    // is genuinely new - see RateEstimator's header comment.
    static raven::RateEstimator climbRateEstimator(0.3f);
    static int64_t t_prev_baro = esp_timer_get_time();

    for(;;) {
        bool bmpsuccess = false;

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            bmpsuccess = readBMP280(BMP280calib, bmp_raw, bmp_out);
            xSemaphoreGive(i2cMutex);
        }

        if (bmpsuccess) {
            const int64_t t_now = esp_timer_get_time();
            const float dt_baro = (float)(t_now - t_prev_baro) * 1.0e-6f;
            t_prev_baro = t_now;

            const float climb_rate = climbRateEstimator.update((float)bmp_out.altitude, dt_baro);

            if (xSemaphoreTake(baroMutex, 10) == pdTRUE) {
                baroSnapshot.valid         = true;
                baroSnapshot.altitude      = (float)bmp_out.altitude;
                baroSnapshot.climb_rate    = climb_rate;
                baroSnapshot.pressure_pa   = (float)bmp_out.pressure;
                baroSnapshot.temperature_c = (float)bmp_out.temperature;
                xSemaphoreGive(baroMutex);
            }
            //printPressTemp(bmp_out);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // 2Hz loop
    }
}

void TaskFlightControl(void *pvParameters) {

    int64_t t_prev = esp_timer_get_time();

    for(;;) {

        // Real dt, not the nominal 10ms - the control loop needs the actual
        // elapsed time (jitter, a missed mutex, scheduling delay all show up
        // here) or the PID D-terms and integrators drift out of sync with
        // reality. flight_kinematics clamps this internally, so an
        // occasional bad cycle can't detonate it.
        const int64_t t_now = esp_timer_get_time();
        const float dt = (float)(t_now - t_prev) * 1.0e-6f;
        t_prev = t_now;

        if (xSemaphoreTake(imuMutex, 10) == pdTRUE) {

            FusionVector gyroscope = { .axis = {gyroC.x, gyroC.y, gyroC.z} };
            FusionVector accelerometer = { .axis = {accelC.x, accelC.y, accelC.z} };
            FusionVector magnetometer = { .axis = {magC.x, magC.y, magC.z} };

            xSemaphoreGive(imuMutex);

            gyroscope = FusionBiasUpdate(&bias, gyroscope);

            // Fusion's own gyro integration otherwise assumes a fixed 10ms
            // tick (settings.sampleRate=100.0f) regardless of actual
            // scheduling jitter - reuse the same measured dt here.
            FusionAhrsSetSamplePeriod(&ahrs, dt);
            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);

            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            // --- gather raw sensors as plain floats. All frame-convention
            // correction, VehicleState construction, and air-density math
            // happens inside flight_kinematics.cpp's runCycle() now - not here.
            raven::RawSensors raw;
            raw.fusion_roll_deg   = euler.angle.roll;
            raw.fusion_pitch_deg  = euler.angle.pitch;
            raw.fusion_yaw_deg    = euler.angle.yaw;
            raw.fusion_gyro_x_dps = gyroscope.axis.x;
            raw.fusion_gyro_y_dps = gyroscope.axis.y;
            raw.fusion_gyro_z_dps = gyroscope.axis.z;

            if (xSemaphoreTake(baroMutex, 5) == pdTRUE) {
                raw.baro_valid         = baroSnapshot.valid;
                raw.baro_altitude      = baroSnapshot.altitude;
                raw.baro_climb_rate    = baroSnapshot.climb_rate;
                raw.baro_pressure_pa   = baroSnapshot.pressure_pa;
                raw.baro_temperature_c = baroSnapshot.temperature_c;
                xSemaphoreGive(baroMutex);
            }

            if (xSemaphoreTake(gnssMutex, 5) == pdTRUE) {
                raw.gnss_fix_valid = (masterGNSSData.fixQuality > 0);
                xSemaphoreGive(gnssMutex);
            }
            // raw.nav_ready / distance_to_go / ground_speed stay at their
            // RawSensors defaults (false / 0) - no guidance source exists
            // yet. Until nav_ready is set from real F-Code target data, the
            // mode machine cannot leave HOVER (fail-safe, not silently wrong).

            raven::Setpoints sp{};
            if (xSemaphoreTake(setpointMutex, 5) == pdTRUE) {
                sp = fkSetpoints;
                xSemaphoreGive(setpointMutex);
            }

            raven::VehicleState st;   // filled by runCycle(), useful for the print below
            raven::ActuatorCmd  out = fk.runCycle(dt, raw, sp, &st);

            // TODO: this is where ESC/servo output goes - not wired yet.
            Serial.printf("mode=%-10s alpha=%5.1f  roll=%6.2f pitch=%6.2f yaw=%6.2f  thrL=%.3f thrR=%.3f  nacL=%5.1f nacR=%5.1f  ail=%+.2f ele=%+.2f rud=%+.2f\n",
                raven::flightModeName(out.mode), out.alpha_cmd * 57.2958f,
                st.roll * 57.2958f, st.pitch * 57.2958f, st.yaw * 57.2958f,
                out.thrust_left, out.thrust_right,
                out.nacelle_left * 57.2958f, out.nacelle_right * 57.2958f,
                out.aileron, out.elevator, out.rudder);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void setup() {

    Serial.begin(115200);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();

    Wire.begin();
    Wire.setClock(400000);

    GNSS.setRxBufferSize(1024); 
    GNSS.begin(9600);
    GNSS.setTimeout(10);

    gnssMutex     = xSemaphoreCreateMutex();
    imuMutex      = xSemaphoreCreateMutex();
    i2cMutex      = xSemaphoreCreateMutex();
    baroMutex     = xSemaphoreCreateMutex();
    setpointMutex = xSemaphoreCreateMutex();
    
    readCalibrationData280(BMP280calib);
    printCalibration280(BMP280calib);
    initBMP280();

    setMPU(setMPU_DLPF(MPU_DLPF_2), setGyroFS(MPU_GYRO_FS_250), setAccFS(MPU_ACCEL_FS_2));
    setMag(magConfRegA(MAG_SAMPLE_AVG_8, MAG_OUTPUT_RATE_75), magConfRegB(MAG_SENS_1370), magModeReg(MAG_CONTINUOUS_MEAS));
    measureGyroOffset(accel, gyro, gyroOffset);


    FusionBiasInitialise(&bias);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .sampleRate = 100.0f,                  
        .convention = FusionConventionNwu,     
        .gain = 0.5f,   // Default filter gain
        .gyroscopeRange = 250.0f,              
        .accelerationRejection = 10.0f,        
        .magneticRejection = 10.0f,            
        .rejectionTimeout = 5.0f,              
    };

    FusionAhrsSetSettings(&ahrs, &settings);

    fk.begin(fkConfig, fkGains);
    fk.setArmed(true);   // TODO: gate this on a real arming switch/sequence,
                          // not unconditionally true at boot, before this
                          // flies anything with props on.

    xTaskCreatePinnedToCore(TaskGNSS, "GNSS", 4096, NULL, 1, NULL, 0);
    
    xTaskCreatePinnedToCore(TaskIMU, "IMU", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(TaskBMP, "Barometer", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(TaskFlightControl, "PID", 8192, NULL, 3, NULL, 1); 

    vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL); 
}
