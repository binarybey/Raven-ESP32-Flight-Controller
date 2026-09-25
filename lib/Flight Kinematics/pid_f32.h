// pid_f32.h - minimal single-precision PID for the Raven flight controller.
//
// Single-precision only: the ESP32 FPU is 32-bit. A stray `double` literal
// (e.g. `0.5` instead of `0.5f`) silently promotes the whole expression to
// software-emulated double and costs you ~50x. Keep every constant `f`-suffixed.
//
// Features:
//   - derivative on measurement (no setpoint-step kick)
//   - first-order low-pass on the derivative term
//   - anti-windup by conditional integration: the integrator freezes while
//     the output is saturated and the error pushes further into the stop,
//     plus an absolute clamp (i_limit). There is no back-calculation term.
//   - explicit output clamp

#pragma once

#include <math.h>
#include <stdbool.h>

namespace raven {

struct PidGains {
    float kp;
    float ki;
    float kd;
    float i_limit;    // absolute clamp on the integral accumulator (output units)
    float d_cutoff;   // Hz, low-pass on the derivative path
    float out_min;
    float out_max;

    // Explicit constructor, not "= value" default member initializers.
    // Reason: a class with NSDMI is NOT an aggregate under C++11 (that rule
    // relaxed in C++14, N3653) - so positional brace-init like
    // `PidGains{22.0f, 6.0f, 0.6f, 30.0f, 25.0f, -60.0f, 60.0f}` falls back to
    // constructor overload resolution and fails to compile under
    // -std=gnu++11 (ESP32 Arduino's default) with "no matching constructor",
    // even though it compiles fine under C++14/17. This constructor, with
    // default arguments reproducing the old NSDMI values, gives identical
    // behavior for both `PidGains()` and `PidGains{...}` call sites under any
    // standard from C++11 up - don't revert to "= value" field defaults.
    PidGains(float kp_ = 0.0f, float ki_ = 0.0f, float kd_ = 0.0f,
             float i_limit_ = 1.0f, float d_cutoff_ = 20.0f,
             float out_min_ = -1.0f, float out_max_ = 1.0f)
        : kp(kp_), ki(ki_), kd(kd_), i_limit(i_limit_), d_cutoff(d_cutoff_),
          out_min(out_min_), out_max(out_max_) {}
};

class Pid {
  public:
    void setGains(const PidGains &g) { g_ = g; }
    const PidGains &gains() const { return g_; }

    void reset() {
        integ_      = 0.0f;
        meas_prev_  = 0.0f;
        d_filt_     = 0.0f;
        primed_     = false;
    }

    // setpoint / measurement in the same units; returns a clamped output.
    // `hold_integrator` freezes the I term (use during saturation or mode holds).
    float update(float setpoint, float measurement, float dt, bool hold_integrator = false) {
        const float err = setpoint - measurement;

        // --- P ---
        const float p = g_.kp * err;

        // --- D on measurement, negated, then low-passed ---
        float d_raw = 0.0f;
        if (primed_ && dt > 0.0f) {
            d_raw = -(measurement - meas_prev_) / dt;
        }
        meas_prev_ = measurement;
        primed_    = true;

        if (g_.d_cutoff > 0.0f && dt > 0.0f) {
            const float rc    = 1.0f / (2.0f * 3.14159265f * g_.d_cutoff);
            const float alpha = dt / (rc + dt);
            d_filt_ += alpha * (d_raw - d_filt_);
        } else {
            d_filt_ = d_raw;
        }
        const float d = g_.kd * d_filt_;

        // --- I with conditional integration ---
        const float unsat = p + integ_ + d;
        const bool  pushing_into_stop =
            (unsat >= g_.out_max && err > 0.0f) || (unsat <= g_.out_min && err < 0.0f);

        if (!hold_integrator && !pushing_into_stop) {
            integ_ += g_.ki * err * dt;
            if (integ_ >  g_.i_limit) integ_ =  g_.i_limit;
            if (integ_ < -g_.i_limit) integ_ = -g_.i_limit;
        }

        float out = p + integ_ + d;
        if (out > g_.out_max) out = g_.out_max;
        if (out < g_.out_min) out = g_.out_min;
        return out;
    }

    // Seed the integrator so the controller hands over bumplessly on mode change.
    void preload(float output_now) {
        integ_ = output_now;
        if (integ_ >  g_.i_limit) integ_ =  g_.i_limit;
        if (integ_ < -g_.i_limit) integ_ = -g_.i_limit;
    }

    float integrator() const { return integ_; }

  private:
    PidGains g_;
    float    integ_     = 0.0f;
    float    meas_prev_ = 0.0f;
    float    d_filt_    = 0.0f;
    bool     primed_    = false;
};

}  // namespace raven