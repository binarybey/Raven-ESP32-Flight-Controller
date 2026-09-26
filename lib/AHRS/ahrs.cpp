#include <Arduino.h>
#include "ahrs.h"
#include "Fusion.h"

static FusionAhrs   ahrs;
static FusionBias   bias;
static FusionVector lastGyro = FUSION_VECTOR_ZERO;   // bias-corrected, deg/s
static bool imuFresh = false;
static bool magFresh = false;

void initAHRS(float sampleRateHz) {
    FusionBiasInitialise(&bias);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .sampleRate = sampleRateHz,
        .convention = FusionConventionNwu,
        .gain = 0.5f,   // Default filter gain
        .gyroscopeRange = 250.0f,
        .accelerationRejection = 10.0f,
        .magneticRejection = 10.0f,
        .rejectionTimeout = 5.0f,
    };
    FusionAhrsSetSettings(&ahrs, &settings);
}

void updateAHRS(const ImuSample &sample, bool gotImu, int64_t now_us, float dt) {
    imuFresh = gotImu && (now_us - sample.imu_us) < IMU_FRESH_US;
    magFresh = (now_us - sample.mag_us) < MAG_FRESH_US;
    if (!imuFresh) return;

    const FusionVector gyroscope     = { .axis = {sample.gyro.x,  sample.gyro.y,  sample.gyro.z} };
    const FusionVector accelerometer = { .axis = {sample.accel.x, sample.accel.y, sample.accel.z} };
    const FusionVector magnetometer  = { .axis = {sample.mag.x,   sample.mag.y,   sample.mag.z} };

    lastGyro = FusionBiasUpdate(&bias, gyroscope);

    // Fusion otherwise assumes exactly the configured sample rate; use the
    // measured period (bounded) for the gyro integration.
    FusionAhrsSetSamplePeriod(&ahrs, constrain(dt, 0.002f, 0.05f));
    if (magFresh) FusionAhrsUpdate(&ahrs, lastGyro, accelerometer, magnetometer);
    else          FusionAhrsUpdateNoMagnetometer(&ahrs, lastGyro, accelerometer);
}

void ahrsToRaw(raven::RawSensors &raw) {
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    raw.imu_valid         = imuFresh;
    raw.mag_valid         = magFresh;
    raw.ahrs_ready        = !FusionAhrsGetFlags(&ahrs).startup;
    raw.fusion_roll_deg   = euler.angle.roll;
    raw.fusion_pitch_deg  = euler.angle.pitch;
    raw.fusion_yaw_deg    = euler.angle.yaw;
    raw.fusion_gyro_x_dps = lastGyro.axis.x;
    raw.fusion_gyro_y_dps = lastGyro.axis.y;
    raw.fusion_gyro_z_dps = lastGyro.axis.z;
}
