// ahrs.h - attitude from the IMU via x-io Fusion (NWU convention).
//
// Hands Fusion's RAW attitude to the flight code; the NWU -> aviation sign
// flips and the magnetic declination are applied inside
// FlightKinematics::runCycle(), nowhere else.

#pragma once

#include <stdint.h>
#include "IMU.h"
#include "flight_kinematics.h"

#define IMU_FRESH_US 30000   // older than 3 samples (100 Hz) = IMU not reporting
#define MAG_FRESH_US 50000   // HMC5883L runs at 75 Hz

// Call once in setup(), after the gyro offset is measured.
void initAHRS(float sampleRateHz);

// One update per control cycle. gotImu: TaskIMU signalled a new sample this
// cycle. The magnetometer is used only if its reading is fresh; otherwise
// the update runs gyro + accelerometer only. dt: measured cycle time (s).
void updateAHRS(const ImuSample &sample, bool gotImu, int64_t now_us, float dt);
//  usage updateAHRS(imu, gotImu, esp_timer_get_time(), dt)

// Attitude, rates and health flags from the last update -> the flight
// code's inputs.
void ahrsToRaw(raven::RawSensors &raw);
