// flight_kinematics.h - tiltrotor (V-22 style) control kinematics and allocation.
//
// ============================================================================
// CONVENTIONS  (read this before touching any sign)
// ============================================================================
//
// Body frame:  x forward, y right, z DOWN.
// Moments:     L about +x = right wing down
//              M about +y = nose up
//              N about +z = nose right
// Euler:       ZYX (yaw-pitch-roll).
//
// !! SENSOR INPUT NOTE !!  x-io's Fusion library (the AHRS this project uses)
// reports raw orientation in North-West-Up (NWU): body x=forward, y=LEFT,
// z=UP - NOT this file's x-fwd/y-right/z-down convention. Roll is unaffected
// (rotation about the shared x-axis), but Fusion's raw pitch is
// nose-DOWN-positive and raw yaw is nose-LEFT-positive - both inverted from
// what this file expects internally. This is NOT a Fusion-specific quirk: it
// is a property of which physical direction is labelled "y" and "z", so no
// choice of Euler-extraction formula or library can undo it (verified this
// two ways - against the compiled Fusion library directly, and again against
// an independent 321-sequence DCM implementation on the identical rotation;
// both report the identical inverted numbers).
//
// The correction is applied ONCE, inside FlightKinematics::runCycle() (see
// flight_kinematics.cpp) - pass it Fusion's raw FusionEuler/gyro fields as
// plain floats via the RawSensors struct below, and it produces a correctly-
// signed VehicleState internally before calling update(). This file has no
// Fusion/Arduino dependency; runCycle()'s implementation is the only place
// that convention boundary is crossed. Do not populate VehicleState by hand
// from raw Fusion output elsewhere - go through runCycle().
//
// alpha = nacelle angle measured FROM VERTICAL.
//         alpha = 0      -> rotors point up, helicopter mode
//         alpha = PI/2   -> rotors point forward, airplane mode
//
// Rotor i sits at r_i = (l_x, +/- l_y, -l_z) relative to CG, and produces
//         F_i = T_i * ( sin(alpha_i), 0, -cos(alpha_i) )
//
// From M = sum( r_i x F_i ):
//         F_up = sum T_i cos(alpha_i)
//         F_x  = sum T_i sin(alpha_i)
//         L    = l_y ( T_L cos(alpha_L) - T_R cos(alpha_R) )
//         M    = -l_z sum T_i sin(alpha_i) + l_x sum T_i cos(alpha_i)
//         N    = l_y ( T_L sin(alpha_L) - T_R sin(alpha_R) )
//
// Everything in `buildEffectiveness()` is the Jacobian of those five lines.
// If you change the airframe layout, change the Jacobian - not the gains.
//
// Units: rad, rad/s, N, N*m, m, m/s, Pa, kg, s. All float.
//
// ============================================================================
// CONTROL STRUCTURE
// ============================================================================
//
//   guidance (distance-to-go, altitude, airspeed)
//        |
//        v  outer loops, blended by mu = clamp(q_dyn / q_ref)
//   attitude demands  (roll_sp, pitch_sp, yaw_rate_sp)
//        |
//        v  Euler kinematic transform (signed cos/sin of bank angle)
//   body rate setpoints (p_sp, q_sp, r_sp)
//        |
//        v  rate PIDs -> angular ACCELERATION demand
//   objective vector v = [F_up, F_x, L, M, N]        (N and N*m)
//        |
//        v  weighted damped least squares against B(alpha, T, q)
//   effectors u = [T_total, dT, da_pitch, da_yaw, aileron, elevator, rudder]
//
// The rate PIDs output rad/s^2 and are multiplied by the inertia tensor. That
// makes the gains physically meaningful and mode-independent - grab Ixx/Iyy/Izz
// straight out of Fusion 360 (Properties -> Physical, set the correct material
// densities first, and take the values about the CENTER OF MASS, not the origin).
//
// ============================================================================

#pragma once

#include <stdint.h>

#include "pid_f32.h"

namespace raven {

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------
enum class FlightMode : uint8_t {
    DISARMED   = 0,
    HOVER      = 1,  // alpha parked at 0
    TRANS_FWD  = 2,  // helicopter -> airplane
    FORWARD    = 3,  // alpha parked at 90 deg
    TRANS_BACK = 4,  // airplane -> helicopter
    FAILSAFE   = 5
};

const char *flightModeName(FlightMode m);

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------
struct VehicleState {
    // attitude - AVIATION convention (roll: right-down+, pitch: nose-up+,
    // yaw: nose-right+). Populate via fusion_bridge.h::fillAttitudeFromFusion(),
    // NOT by copying Fusion's raw Euler output directly - see the note above.
    float roll  = 0.0f;   // rad
    float pitch = 0.0f;   // rad
    float yaw   = 0.0f;   // rad

    // body rates, gyro, already low-passed - same convention/bridge as above
    float p = 0.0f, q = 0.0f, r = 0.0f;   // rad/s

    // air data (nose pitot)
    float q_dyn    = 0.0f;    // Pa, differential pressure, zero-offset removed
    float airspeed = 0.0f;    // m/s, = sqrt(2*q_dyn/rho); pass 0 if invalid
    float rho      = 1.225f;  // kg/m^3

    // vertical channel
    float altitude   = 0.0f;  // m
    float climb_rate = 0.0f;  // m/s, positive up

    // navigation
    float distance_to_go = 0.0f;  // m, horizontal range to active waypoint
    float ground_speed   = 0.0f;  // m/s

    bool airspeed_valid = false;
    bool nav_valid      = false;
};

struct Setpoints {
    float roll_sp       = 0.0f;  // rad, bank demand (forward flight / hybrid)
    float pitch_sp      = 0.0f;  // rad, pitch demand (forward flight)
    float yaw_rate_sp   = 0.0f;  // rad/s, EARTH-referenced turn rate
    float climb_rate_sp = 0.0f;  // m/s, positive up
    float airspeed_sp   = 0.0f;  // m/s, forward-flight target
    bool  hold_position = true;  // hover: no translation demand
};

// ---------------------------------------------------------------------------
// Outputs
// ---------------------------------------------------------------------------
struct ActuatorCmd {
    float thrust_left  = 0.0f;   // 0..1 normalised ESC command
    float thrust_right = 0.0f;
    float nacelle_left  = 0.0f;  // rad from vertical, servo-ready
    float nacelle_right = 0.0f;
    float aileron  = 0.0f;       // -1..1, positive = roll right
    float elevator = 0.0f;       // -1..1, positive = nose up
    float rudder   = 0.0f;       // -1..1, positive = nose right

    // ---- telemetry / debug ----
    FlightMode mode = FlightMode::DISARMED;
    float alpha_cmd = 0.0f;      // rad, collective nacelle schedule
    float alpha_min = 0.0f;      // rad, corridor floor at current airspeed
    float alpha_max = 0.0f;      // rad, corridor ceiling at current airspeed
    bool  corridor_limited = false;
    float mu = 0.0f;             // aero authority fraction, 0..1
    float v_demand[5] = {0};     // [F_up, F_x, L, M, N] before allocation
    float u_applied[7] = {0};    // allocator output after saturation
    float wing_lift_est = 0.0f;  // N
    float thrust_total  = 0.0f;  // N
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct VehicleConfig {
    // ---- mass / inertia (Fusion 360 -> Properties, about the CENTER OF MASS) ----
    float mass = 3.5f;      // kg
    float Ixx  = 0.12f;     // kg*m^2, roll
    float Iyy  = 0.18f;     // kg*m^2, pitch
    float Izz  = 0.26f;     // kg*m^2, yaw

    // ---- rotor geometry relative to CG ----
    float l_y = 0.42f;      // m, half the rotor-hub lateral separation
    float l_z = 0.06f;      // m, hub height ABOVE cg (positive = above).
                            //    This alone sets hover pitch authority - see
                            //    hoverPitchAuthority(). If it is tiny, your
                            //    pitch loop will be weak in helicopter mode.
    float l_x = 0.01f;      // m, hub position AHEAD of cg. Near zero for a
                            //    hover-trimmed tiltrotor; a non-zero value
                            //    couples collective thrust into pitch, which
                            //    the allocator will compensate for you.

    // ---- wing / aero ----
    float wing_area    = 0.30f;   // m^2, planform
    float wing_span    = 1.20f;   // m
    float mean_chord   = 0.25f;   // m
    float aspect_ratio = 4.8f;
    float oswald_e     = 0.80f;
    float CL_max       = 1.10f;
    float CL_alpha     = 4.60f;   // per rad
    float CD0          = 0.045f;

    // Control-surface effectiveness, expressed so that a POSITIVE command
    // produces a POSITIVE moment. Handle servo reversal in your PWM layer,
    // not here. Units: moment coefficient per unit command (-1..1).
    float Cl_da = 0.18f;
    float Cm_de = 0.55f;
    float Cn_dr = 0.08f;

    // ---- propulsion ----
    float thrust_max_per_rotor = 26.0f;   // N, static, at full throttle
    float thrust_min_per_rotor = 0.5f;    // N, keep ESCs spinning
    float nacelle_rate_max     = 0.70f;   // rad/s, servo slew limit
    float nacelle_pitch_band   = 0.17f;   // rad (~10 deg) max collective
                                          // perturbation available to pitch
    float nacelle_yaw_band     = 0.14f;   // rad (~8 deg) max differential tilt

    // ---- conversion corridor ----
    float stall_margin  = 1.25f;   // V >= margin * V_stall(alpha)
    float v_max_hover   = 8.0f;    // m/s, airspeed ceiling with nacelles vertical
    float v_max_forward = 34.0f;   // m/s, airframe Vne

    // ---- transition scheduling (HYSTERESIS - a single threshold chatters) ----
    float dist_to_forward   = 400.0f;  // m, beyond this go airplane
    float dist_to_hover     = 250.0f;  // m, inside this come back
    float alpha_rate_nominal = 0.25f;  // rad/s, commanded conversion rate

    // ---- allocation ----
    float q_ref      = 120.0f;   // Pa, dynamic pressure at which control
                                 // surfaces are considered fully effective.
                                 // ~= 0.5*rho*V_cruise^2. Sets `mu`.
    float lambda_reg = 1.0e-4f;  // Tikhonov damping on the 5x5 solve

    // Effector cost weights (higher = allocator avoids using it).
    float w_thrust        = 1.0e-4f;
    float w_dthrust       = 1.0e-3f;
    float w_dalpha_pitch  = 3.0f;
    float w_dalpha_yaw    = 3.0f;
    float w_aileron       = 1.0f;
    float w_elevator      = 1.0f;
    float w_rudder        = 1.0f;

    // Objective priority weights. Raising a row makes the allocator track that
    // axis tighter. F_x is deliberately cheap in hover: pitch and surge share
    // the same effector at alpha=0, and attitude must win that fight.
    float pri_Fup      = 2.0f;
    float pri_Fx_hover = 0.05f;
    float pri_Fx_fwd   = 1.5f;
    float pri_moment   = 4.0f;

    // ---- hover command authority ----
    // The stabiliser ALWAYS regulates roll and pitch. These flags only control
    // whether the pilot/guidance may command a non-zero attitude in hover.
    bool  hover_allow_roll_cmd  = false;
    bool  hover_allow_pitch_cmd = false;
    float hover_attitude_limit  = 0.17f;  // rad, clamp if the above are enabled

    // ---- misc ----
    float g = 9.80665f;
};

// ---------------------------------------------------------------------------
// Gain set
// ---------------------------------------------------------------------------
struct ControlGains {
    // outer attitude -> euler rate demand (proportional, rad/s per rad)
    float kp_roll_att  = 5.0f;
    float kp_pitch_att = 5.0f;

    // inner body rate -> angular acceleration demand (rad/s^2)
    PidGains rate_roll{ 22.0f, 6.0f, 0.6f, 30.0f, 25.0f, -60.0f, 60.0f };
    PidGains rate_pitch{ 20.0f, 6.0f, 0.6f, 30.0f, 25.0f, -50.0f, 50.0f };
    PidGains rate_yaw{ 14.0f, 4.0f, 0.2f, 20.0f, 20.0f, -35.0f, 35.0f };

    // helicopter-mode vertical: climb rate -> vertical accel demand (m/s^2)
    PidGains climb_hover{ 3.0f, 1.2f, 0.10f, 6.0f, 8.0f, -6.0f, 8.0f };

    // airplane-mode vertical: climb rate -> pitch demand (rad)
    PidGains climb_fwd{ 0.055f, 0.020f, 0.0f, 0.25f, 8.0f, -0.30f, 0.35f };

    // airplane-mode speed: airspeed error -> longitudinal accel demand (m/s^2)
    PidGains speed_fwd{ 1.2f, 0.45f, 0.0f, 5.0f, 5.0f, -5.0f, 6.0f };
};

// ---------------------------------------------------------------------------
// RateEstimator - low-pass derivative of a slowly-sampled signal (e.g.
// barometer altitude at ~2Hz). Call update() only when a genuinely NEW
// sample has arrived - calling it every control-loop tick with a repeated
// stale value will bias the estimate toward zero between samples. Owned and
// driven by whichever task reads that sensor (see main.cpp's TaskBMP), not
// by FlightKinematics itself, since only that task knows when a sample is new.
// ---------------------------------------------------------------------------
class RateEstimator {
  public:
    // cutoff_hz: low-pass corner on the derivative. Set below half the
    // sensor's own sample rate (e.g. <1Hz for a 2Hz barometer).
    explicit RateEstimator(float cutoff_hz = 0.5f) : cutoff_hz_(cutoff_hz) {}

    void reset() { primed_ = false; rate_ = 0.0f; }

    // value: new sample. dt: seconds since the LAST call to update() (not
    // since the last control-loop tick). Returns the filtered rate.
    float update(float value, float dt) {
        if (primed_ && dt > 1.0e-4f) {
            const float raw = (value - prev_value_) / dt;
            if (cutoff_hz_ > 0.0f) {
                const float rc    = 1.0f / (2.0f * 3.14159265f * cutoff_hz_);
                const float alpha = dt / (rc + dt);
                rate_ += alpha * (raw - rate_);
            } else {
                rate_ = raw;
            }
        }
        prev_value_ = value;
        primed_     = true;
        return rate_;
    }

    float rate() const { return rate_; }

  private:
    float cutoff_hz_;
    float prev_value_ = 0.0f;
    float rate_       = 0.0f;
    bool  primed_     = false;
};

// ---------------------------------------------------------------------------
// RawSensors - everything FlightKinematics::runCycle() needs, as plain
// floats/bools only. No Fusion.h, no Arduino.h, no project-specific sensor
// struct types - this is the ONE boundary where hardware/library types get
// unpacked into plain data before crossing into this module. main.cpp reads
// its sensors (however it reads them, whatever mutexes that needs) and fills
// this struct; runCycle() does the rest, including the NWU sign correction.
// ---------------------------------------------------------------------------
struct RawSensors {
    // --- attitude: Fusion's RAW output, uncorrected. Pass euler.angle.roll/
    // pitch/yaw and the BIAS-CORRECTED gyroscope vector (the one actually fed
    // to FusionAhrsUpdate) directly - do not pre-negate anything yourself.
    float fusion_roll_deg  = 0.0f;
    float fusion_pitch_deg = 0.0f;
    float fusion_yaw_deg   = 0.0f;
    float fusion_gyro_x_dps = 0.0f;
    float fusion_gyro_y_dps = 0.0f;
    float fusion_gyro_z_dps = 0.0f;

    // --- barometer ---
    bool  baro_valid      = false;
    float baro_altitude   = 0.0f;   // m
    float baro_climb_rate = 0.0f;   // m/s, positive up - compute via RateEstimator
                                     // at the barometer's own sample rate, not here
    float baro_pressure_pa    = 101325.0f;  // Pa, for air-density calc
    float baro_temperature_c  = 15.0f;      // deg C, for air-density calc

    // --- GNSS ---
    // nav_valid is deliberately NOT derived from fix quality alone - it gates
    // the transition mode machine, which needs a MEANINGFUL distance_to_go
    // (i.e. a guidance target), not just "GPS has a fix". Leave gnss_fix_valid
    // as raw status for logging; set distance_to_go/ground_speed/nav_ready
    // once a guidance source (F-Code) supplies an actual target.
    bool  gnss_fix_valid    = false;
    float gnss_ground_speed = 0.0f;  // m/s, 0 until wired to a guidance target

    // --- guidance (from whatever parses F-Code) ---
    bool  nav_ready      = false;   // true once a real target + distance are known
    float distance_to_go = 0.0f;    // m

    // --- air data - no pitot driver exists yet; stays false/default until one does ---
    bool  airspeed_valid = false;
    float airspeed = 0.0f;
    float q_dyn    = 0.0f;
};

// ---------------------------------------------------------------------------
// Main class
// ---------------------------------------------------------------------------
class FlightKinematics {
  public:
    void begin(const VehicleConfig &cfg, const ControlGains &gains);

    // Call from the core-1 control task at a fixed rate (200-400 Hz).
    // `dt` is sanitised internally but feed it from esp_timer_get_time().
    void update(float dt, const VehicleState &st, const Setpoints &sp, ActuatorCmd &out);

    // Single entry point for real hardware use: takes plain sensor floats
    // (see RawSensors above), applies the NWU->internal-convention correction,
    // builds VehicleState, calls update(), returns the command. This is what
    // main.cpp should call every cycle - it should not construct VehicleState
    // by hand. Also exposes the corrected VehicleState via `out_state` (pass
    // nullptr if you don't need it) for telemetry/logging.
    ActuatorCmd runCycle(float dt, const RawSensors &raw, const Setpoints &sp,
                         VehicleState *out_state = nullptr);

    void setArmed(bool armed);
    void requestFailsafe();
    void reset();   // zero all integrators and latched state

    FlightMode mode() const { return mode_; }
    float      alpha() const { return alpha_cmd_; }

    // ---- analysis helpers, safe to call from telemetry on core 0 ----

    // Conversion corridor at a given true airspeed.
    void corridor(float V, float rho, float &alpha_min, float &alpha_max) const;

    // Steady-state dynamic pressure the airframe will settle at for a given
    // nacelle angle and total thrust, INCLUDING induced drag. Returns < 0 if no
    // equilibrium exists (not enough thrust to overcome drag at that alpha).
    //   CD0*S*q^2 - T*sin(a)*q + L_wing^2/(S*pi*AR*e) = 0,  L_wing = m*g - T*cos(a)
    float expectedDynamicPressure(float alpha, float thrust_total) const;

    // Total thrust needed to hold a given alpha at a given airspeed in level flight.
    float trimThrust(float alpha, float V, float rho) const;

    // Pitch moment available per radian of collective nacelle perturbation.
    // In hover this collapses to T_total * l_z. Sanity-check it against
    // Iyy * (desired pitch acceleration). N*m per rad.
    float hoverPitchAuthority(float thrust_total) const;

  private:
    void  updateMode(const VehicleState &st);
    float scheduleAlpha(float dt, const VehicleState &st, ActuatorCmd &out);
    void  buildEffectiveness(float alpha, float thrust_total, float q_dyn, float B[5][7]) const;
    void  allocate(const float B[5][7], const float v[5], const float pri[5], float u[7]);
    float wingLift(const VehicleState &st) const;

    VehicleConfig cfg_;
    ControlGains  gains_;

    Pid pid_rate_roll_, pid_rate_pitch_, pid_rate_yaw_;
    Pid pid_climb_hover_, pid_climb_fwd_, pid_speed_fwd_;

    FlightMode mode_          = FlightMode::DISARMED;
    float      alpha_cmd_     = 0.0f;   // rad, collective schedule (rate limited)
    float      alpha_target_  = 0.0f;   // rad, where the state machine wants to be
    float      thrust_total_  = 0.0f;   // N, last cycle, seeds the Jacobian
    float      nac_left_prev_ = 0.0f;
    float      nac_right_prev_= 0.0f;
    bool       armed_         = false;
    bool       failsafe_      = false;
};

}  // namespace raven