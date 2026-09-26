#include <Arduino.h>
#include "air_data.h"

PitotFilter::PitotFilter(float sampleRateHz, float cutoffHz, float zeroSeconds) {
    const float dt = 1.0f / sampleRateHz;
    alpha_       = dt / (1.0f / (2.0f * 3.14159265f * cutoffHz) + dt);
    zeroSamples_ = (int)(zeroSeconds * sampleRateHz + 0.5f);
}

bool PitotFilter::update(float dp) {
    if (!zeroed_) {
        zeroSum_ += dp;
        if (++zeroCount_ >= zeroSamples_) {
            offset_ = (float)(zeroSum_ / zeroCount_);
            zeroed_ = true;
            Serial.printf("Pitot zeroed: offset %.1f Pa\n", offset_);
        }
        return false;
    }
    filtered_ += alpha_ * ((dp - offset_) - filtered_);
    return true;
}

void pitotToRaw(const AirSnapshot &air, bool present, int64_t now_us, raven::RawSensors &raw) {
    raw.pitot_valid = present && air.zeroed &&
                      (now_us - air.sample_us) < PITOT_FRESH_US &&
                      air.q_pa > -50.0f && air.q_pa < 5000.0f;   // reversed ports / garbage
    raw.pitot_q_pa  = air.q_pa;
}
