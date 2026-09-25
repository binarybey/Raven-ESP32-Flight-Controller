// hardware_interface.h - the plug-in contract between the flight code and
// the not-yet-wired hardware: ESCs + servos (outputs) and the pitot tube
// (input). The flight code only ever talks to these two abstract classes;
// your hardware libraries implement them and main.cpp points at your
// implementation. Nothing else in the flight code changes when the hardware
// arrives.
//
// Until then, main.cpp uses the Null* backends at the bottom of this file:
// outputs are computed and logged but go nowhere, and there is no airspeed
// (the flight code then refuses to convert out of hover - see
// FlightKinematics::scheduleAlpha()).
//
// PLUGGING IN (main.cpp, section "Hardware backends"):
//     #include "your_servo_esc_lib.h"
//     static YourActuators yourActuators;          // : public raven::ActuatorBackend
//     raven::ActuatorBackend *actuators = &yourActuators;
// and the same for raven::AirDataBackend.
//
// This header deliberately has no Arduino/FreeRTOS dependency.

#pragma once

namespace raven {

// ===========================================================================
// OUTPUTS - ESCs and servos
// ===========================================================================
//
// Universal format: physical angles for the nacelles, normalised -1..1 for
// the control surfaces, normalised 0..1 for throttle. Your library owns ALL
// hardware specifics: pulse widths, servo direction/reversal, trims, end
// points, ESC protocol (PWM/OneShot/DShot), and clamping to what the
// mechanism can physically reach.
struct ActuatorOutputs {
    // false = DISARMED. ESCs must output their stop / disarmed signal no
    // matter what throttle_* say. Servos should keep tracking their commands
    // (the nacelles are still driven while disarmed so you can bench-check
    // them).
    bool  armed = false;

    // Motor command, 0..1. 0 = stopped/ESC minimum, 1 = full throttle.
    // Already linearised for thrust by the flight code (thrust ~ throttle^2
    // assumption until a measured thrust curve replaces it), so map it
    // LINEARLY to the ESC range (e.g. 1000..2000 us). While armed the flight
    // code never sends less than the idle value set by
    // VehicleConfig::thrust_min_per_rotor, so props keep spinning.
    float throttle_left  = 0.0f;
    float throttle_right = 0.0f;

    // Nacelle tilt, radians measured FROM VERTICAL:
    //   0      = rotor axis straight up   (helicopter / hover)
    //   +pi/2  = rotor axis straight forward (airplane)
    // Commanded range is [-0.10, pi/2 + 0.10] rad. Map it with a per-side
    // two-point calibration (pulse at 0 rad, pulse at pi/2 rad) and clamp to
    // the mechanism's real limits. Already slew-limited by the flight code
    // (VehicleConfig::nacelle_rate_max).
    float nacelle_left_rad  = 0.0f;
    float nacelle_right_rad = 0.0f;

    // Control surfaces, -1..1, signed by the MOMENT they produce:
    //   aileron  +1 = roll right  (right aileron trailing edge UP, left DOWN)
    //   elevator +1 = nose up     (trailing edge UP)
    //   rudder   +1 = nose right  (trailing edge to the RIGHT, seen from behind)
    // +/-1 = full mechanical throw. Handle servo reversal and trim in your
    // library, never by flipping signs in the flight code.
    float aileron  = 0.0f;
    float elevator = 0.0f;
    float rudder   = 0.0f;
};

class ActuatorBackend {
  public:
    virtual ~ActuatorBackend() {}

    // Called once from setup(), before any write(). Put the ESCs into their
    // disarmed state here. Return false if the hardware isn't responding.
    virtual bool begin() = 0;

    // Called EVERY control cycle (100 Hz, the IMU rate) from the flight-
    // control task on core 1 - the highest-priority task in the system.
    // Rules:
    //   - Return quickly (target < 200 us): latch the values into hardware
    //     (LEDC/MCPWM/RMT registers) and return. No delay(), no Serial.
    //   - Called with armed=false continuously while disarmed.
    //   - If your outputs go over the shared I2C bus (e.g. a PCA9685), take
    //     `i2cMutex` (declared in main.cpp) with a short timeout (<= 2 ms)
    //     and skip the update if you don't get it. Direct PWM pins are
    //     preferred - they need no bus at all.
    //   - Strongly recommended: an output watchdog in your library - if
    //     write() hasn't been called for ~100 ms (flight task hung), drive
    //     the ESCs to their stop signal on your own.
    virtual void write(const ActuatorOutputs &out) = 0;
};

// ===========================================================================
// INPUT - pitot / differential pressure
// ===========================================================================
//
// Universal format: differential pressure in pascals, signed, with the
// sensor's own transfer function (datasheet counts -> Pa) already applied,
// POSITIVE when the pitot (total-pressure) port is above the static port.
// Do NOT zero it yourself - the firmware averages the first 2 s after
// power-up as the zero offset (keep the pitot still and out of the wind, or
// covered, during boot), filters it, checks it, and converts to airspeed
// using live air density from the BMP280.
//
// Suggested sensor range: at the 34 m/s placeholder Vne, q is ~700 Pa, so a
// full scale of at least ~1 kPa (e.g. MS4525DO +/-1 psi, 6.9 kPa) - a
// 500 Pa part would clip in fast forward flight.
class AirDataBackend {
  public:
    virtual ~AirDataBackend() {}

    // Called once from setup(). Return false if no sensor responds - the
    // firmware then runs without airspeed (hover only).
    virtual bool begin() = 0;

    // Called at 50 Hz from a core-0 task WITH `i2cMutex` ALREADY HELD (do not
    // take it again). Return true and fill diff_pressure_pa when a NEW
    // sample was read; return false if no new sample is ready or the read
    // failed (don't repeat an old value). Must return within ~2 ms.
    // The firmware marks airspeed invalid if no new sample arrives for
    // 200 ms.
    virtual bool read(float &diff_pressure_pa) = 0;
};

// ---------------------------------------------------------------------------
// Placeholders until the real hardware exists.
// ---------------------------------------------------------------------------
class NullActuatorBackend : public ActuatorBackend {
  public:
    bool begin() override { return true; }
    void write(const ActuatorOutputs &) override {}
};

class NullAirDataBackend : public AirDataBackend {
  public:
    bool begin() override { return false; }   // "no pitot fitted"
    bool read(float &) override { return false; }
};

}  // namespace raven
