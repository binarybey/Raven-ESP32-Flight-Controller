// air_data.h - pitot processing: zero-offset, low-pass, and the hand-off to
// the flight code. The sensor itself is read through the AirDataBackend in
// lib/Hardware Interface (pitot library).

#pragma once

#include <stdint.h>
#include "flight_kinematics.h"

#define PITOT_FRESH_US 200000   // no new sample for 200 ms = airspeed invalid

// Latest pitot result - TaskAirData fills it, TaskFlightControl reads it
// (under airMutex).
struct AirSnapshot {
    bool    zeroed;      // the zero offset has been measured
    float   q_pa;        // Pa, differential pressure, zero removed, filtered
    int64_t sample_us;   // esp_timer time of the sample
};

// Averages the first zeroSeconds of samples as the zero offset (keep the
// pitot still and covered at power-up), then low-passes.
class PitotFilter {
  public:
    PitotFilter(float sampleRateHz, float cutoffHz = 5.0f, float zeroSeconds = 2.0f);

    // Feed one NEW sample (Pa, as the backend returns it). Returns true once
    // zeroing is done, i.e. q() holds a valid filtered value.
    bool update(float diff_pressure_pa);

    float q() const      { return filtered_; }
    float offset() const { return offset_; }

  private:
    float  alpha_;
    int    zeroSamples_;
    int    zeroCount_ = 0;
    double zeroSum_   = 0.0;
    float  offset_    = 0.0f;
    float  filtered_  = 0.0f;
    bool   zeroed_    = false;
};

// Pitot snapshot -> the flight code's airspeed input: valid only if a pitot
// is present, zeroed, fresh, and plausible.
void pitotToRaw(const AirSnapshot &air, bool present, int64_t now_us, raven::RawSensors &raw);
