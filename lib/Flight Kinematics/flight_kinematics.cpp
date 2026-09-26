// flight_kinematics.cpp - see flight_kinematics.h for conventions and the
// derivation of the effectiveness matrix.

#include "flight_kinematics.h"

#include <math.h>

namespace raven {

// ---------------------------------------------------------------------------
// small float helpers
// ---------------------------------------------------------------------------
static const float kPi       = 3.14159265f;
static const float kHalfPi   = 1.57079633f;
static const float kTwoPi    = 6.28318531f;
static const float kDegToRad = 0.017453292f;

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline float wrapPi(float a) {
    while (a >  kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}

const char *flightModeName(FlightMode m) {
    switch (m) {
        case FlightMode::DISARMED:   return "DISARMED";
        case FlightMode::HOVER:      return "HOVER";
        case FlightMode::TRANS_FWD:  return "TRANS_FWD";
        case FlightMode::FORWARD:    return "FORWARD";
        case FlightMode::TRANS_BACK: return "TRANS_BACK";
        case FlightMode::FAILSAFE:   return "FAILSAFE";
    }
    return "?";
}

const char *missionPhaseName(MissionPhase p) {
    switch (p) {
        case MissionPhase::NONE:    return "MANUAL";
        case MissionPhase::TAKEOFF: return "TAKEOFF";
        case MissionPhase::ENROUTE: return "ENROUTE";
        case MissionPhase::LAND:    return "LAND";
    }
    return "?";
}

const char *failsafeReasonName(FailsafeReason r) {
    switch (r) {
        case FailsafeReason::NONE:      return "-";
        case FailsafeReason::COMMANDED: return "COMMANDED";
        case FailsafeReason::NAV_LOST:  return "NAV_LOST";
        case FailsafeReason::IMU:       return "IMU";
        case FailsafeReason::BARO:      return "BARO";
        case FailsafeReason::TERRAIN:   return "TERRAIN";
    }
    return "?";
}

const char *configIssueText(uint32_t bit) {
    switch (bit) {
        case CFG_BAD_MASS_INERTIA:   return "mass or inertia not positive";
        case CFG_HOVER_THRUST:       return "hover needs more than 75% of max thrust (mass vs thrust_max_per_rotor)";
        case CFG_NO_FULL_CONVERSION: return "wing can't carry the weight even at v_max_forward - forward flight stays partially converted (rotors lifting, power hungry)";
        case CFG_CRUISE_BELOW_STALL: return "cruise_airspeed is below stall_margin * wing-only stall speed - rotors carry part of the weight in cruise";
    }
    return "?";
}

ActuatorOutputs toActuatorOutputs(const ActuatorCmd &cmd) {
    ActuatorOutputs hw;
    hw.armed             = cmd.armed;
    hw.throttle_left     = cmd.armed ? cmd.thrust_left  : 0.0f;
    hw.throttle_right    = cmd.armed ? cmd.thrust_right : 0.0f;
    hw.nacelle_left_rad  = cmd.nacelle_left;
    hw.nacelle_right_rad = cmd.nacelle_right;
    hw.aileron           = cmd.aileron;
    hw.elevator          = cmd.elevator;
    hw.rudder            = cmd.rudder;
    return hw;
}

// Effector indices into u[7]
enum : int {
    U_THRUST = 0,   // N, total thrust  (T_L + T_R)
    U_DTHRUST,      // N, differential  (T_L - T_R)
    U_NAC_L,        // rad, LEFT nacelle deviation from the collective schedule
    U_NAC_R,        // rad, RIGHT nacelle deviation from the collective schedule
    U_AILERON,      // -1..1
    U_ELEVATOR,     // -1..1
    U_RUDDER,       // -1..1
    NU = 7
};

// Objective indices into v[5]
enum : int { V_FUP = 0, V_FX, V_L, V_M, V_N, NV = 5 };

// ---------------------------------------------------------------------------
// Cholesky solve for the 5x5 symmetric positive-definite normal equations.
// ~100 flops, single precision, no allocation. Trivial at 100 Hz on an ESP32.
// ---------------------------------------------------------------------------
static bool cholDecompose(const float A[NV][NV], float L[NV][NV]) {
    for (int i = 0; i < NV; ++i) {
        for (int j = 0; j <= i; ++j) {
            float s = A[i][j];
            for (int k = 0; k < j; ++k) s -= L[i][k] * L[j][k];
            if (i == j) {
                if (s <= 1.0e-20f) return false;
                L[i][i] = sqrtf(s);
            } else {
                L[i][j] = s / L[j][j];
            }
        }
        for (int j = i + 1; j < NV; ++j) L[i][j] = 0.0f;
    }
    return true;
}

static void cholSolve(const float L[NV][NV], const float b[NV], float x[NV]) {
    float y[NV];
    for (int i = 0; i < NV; ++i) {
        float s = b[i];
        for (int k = 0; k < i; ++k) s -= L[i][k] * y[k];
        y[i] = s / L[i][i];
    }
    for (int i = NV - 1; i >= 0; --i) {
        float s = y[i];
        for (int k = i + 1; k < NV; ++k) s -= L[k][i] * x[k];
        x[i] = s / L[i][i];
    }
}

// ===========================================================================
// setup, arming, failsafe
// ===========================================================================
uint32_t FlightKinematics::begin(const VehicleConfig &cfg, const ControlGains &gains) {
    cfg_   = cfg;
    gains_ = gains;

    pid_rate_roll_.setGains(gains_.rate_roll);
    pid_rate_pitch_.setGains(gains_.rate_pitch);
    pid_rate_yaw_.setGains(gains_.rate_yaw);
    pid_climb_hover_.setGains(gains_.climb_hover);
    pid_climb_fwd_.setGains(gains_.climb_fwd);
    pid_speed_fwd_.setGains(gains_.speed_fwd);
    pid_heading_hold_.setGains(gains_.heading_hold);
    pid_xtrack_to_latvel_hover_.setGains(gains_.xtrack_to_latvel_hover);
    pid_latvel_to_roll_hover_.setGains(gains_.latvel_to_roll_hover);
    pid_speed_hover_.setGains(gains_.speed_hover);
    pid_xtrack_to_course_fwd_.setGains(gains_.xtrack_to_course_fwd);
    pid_altitude_hold_.setGains(gains_.altitude_hold);

    armed_     = false;
    failsafe_  = false;
    fs_reason_ = FailsafeReason::NONE;
    phase_     = MissionPhase::NONE;
    reset();

    config_issues_ = validateConfig();
    return config_issues_;
}

void FlightKinematics::resetControllers() {
    pid_rate_roll_.reset();
    pid_rate_pitch_.reset();
    pid_rate_yaw_.reset();
    pid_climb_hover_.reset();
    pid_climb_fwd_.reset();
    pid_speed_fwd_.reset();
    pid_heading_hold_.reset();
    pid_xtrack_to_latvel_hover_.reset();
    pid_latvel_to_roll_hover_.reset();
    pid_speed_hover_.reset();
    pid_xtrack_to_course_fwd_.reset();
    pid_altitude_hold_.reset();
}

// Control state only. The position and altitude ESTIMATES (dead-reckoning,
// baro/GNSS fusion, baro bias) are sensor state and keep running across
// arm/disarm - resetting the bias estimate on every arm would throw away
// minutes of GNSS calibration.
void FlightKinematics::reset() {
    resetControllers();

    mode_           = armed_ ? (failsafe_ ? FlightMode::FAILSAFE : FlightMode::HOVER)
                             : FlightMode::DISARMED;
    alpha_cmd_      = 0.0f;
    alpha_target_   = 0.0f;
    thrust_total_   = cfg_.mass * cfg_.g;
    nac_left_prev_  = 0.0f;
    nac_right_prev_ = 0.0f;

    takeoff_hold_set_ = false;
    fs_hold_set_      = false;
    nav_lost_s_       = 0.0f;
    land_descending_  = false;
    land_timer_s_     = 0.0f;
    landed_timer_s_   = 0.0f;
    descend_time_s_   = 0.0f;
    terrain_lost_s_   = 0.0f;
    alt_ref_msl_      = false;
    imu_bad_s_        = 0.0f;
    baro_bad_s_       = 0.0f;
}

bool FlightKinematics::arm(const char **reason) {
    const char *why = nullptr;
    const bool mission = fcode::loaded();

    if (armed_)                                        why = "already armed";
    else if (config_issues_ & CFG_FATAL_MASK)          why = "vehicle config invalid (see boot log)";
    else if (!have_last_)                              why = "no sensor data yet";
    else if (!last_raw_.imu_valid)                     why = "IMU not responding";
    else if (!last_raw_.mag_valid)                     why = "magnetometer not responding";
    else if (!last_raw_.ahrs_ready)                    why = "AHRS still converging - wait a few seconds";
    else if (fabsf(last_st_.roll)  > cfg_.prearm_max_tilt_rad ||
             fabsf(last_st_.pitch) > cfg_.prearm_max_tilt_rad) why = "vehicle not level";
    else if (!last_raw_.baro_valid || !alt_fusion_primed_) why = "barometer not ready";
    else if (mission && fcode::complete())             why = "mission already flown - reboot to reload it";
    else if (mission && !last_st_.nav_valid)           why = "no GNSS position (a mission is loaded)";
    else if (mission && last_raw_.gnss_sats < cfg_.prearm_min_sats) why = "too few GNSS satellites";
    else if (mission && last_raw_.gnss_hdop > cfg_.prearm_max_hdop)  why = "GNSS HDOP too high";
    else if (mission && hypotf(last_st_.pos_x, last_st_.pos_y) > cfg_.prearm_max_start_distance_m)
        why = "too far from the mission start (F90)";
    else if (mission && !mission_terrain_ok_)          why = "DEM tiles missing along the mission route";
    else if (mission && !last_st_.terrain_valid)       why = "no DEM elevation at the current position";
    else if (mission && gnss_alt_settle_s_ < cfg_.prearm_gnss_settle_s)
        why = "altitude calibration settling (GNSS + baro) - wait ~30 s after fix";

    if (why) {
        if (reason) *reason = why;
        return false;
    }

    armed_     = true;
    failsafe_  = false;
    fs_reason_ = FailsafeReason::NONE;
    reset();   // -> HOVER

    // Fallback AGL reference in the baro-only frame - see ground_ref_baro_m_.
    ground_ref_baro_m_ = baro_frame_alt_m_;
    ground_ref_set_    = true;

    phase_ = mission ? MissionPhase::TAKEOFF : MissionPhase::NONE;
    if (reason) *reason = mission ? "armed - mission TAKEOFF"
                                  : "armed - no mission, manual setpoints (hold level)";
    return true;
}

void FlightKinematics::disarm() {
    armed_    = false;
    failsafe_ = false;
    phase_    = MissionPhase::NONE;
    mode_     = FlightMode::DISARMED;
    resetControllers();
}

void FlightKinematics::requestFailsafe(FailsafeReason why) {
    if (!armed_ || failsafe_) return;   // first reason wins; nothing to do on the ground
    failsafe_    = true;
    fs_reason_   = why;
    fs_hold_set_ = false;
    mode_        = FlightMode::FAILSAFE;
}

uint32_t FlightKinematics::validateConfig() const {
    uint32_t issues = 0;
    if (!(cfg_.mass > 0.0f) || !(cfg_.Ixx > 0.0f) || !(cfg_.Iyy > 0.0f) || !(cfg_.Izz > 0.0f))
        issues |= CFG_BAD_MASS_INERTIA;

    const float hover_fraction = cfg_.mass * cfg_.g / (2.0f * cfg_.thrust_max_per_rotor);
    if (!(hover_fraction <= 0.75f)) issues |= CFG_HOVER_THRUST;

    if (corridorCeiling(cfg_.v_max_forward, 1.225f) < kHalfPi - 0.03f) issues |= CFG_NO_FULL_CONVERSION;

    if (cfg_.cruise_airspeed < cfg_.stall_margin * stallSpeed(1.225f))
        issues |= CFG_CRUISE_BELOW_STALL;

    return issues;
}

// ===========================================================================
// analysis helpers
// ===========================================================================
// Ceiling: the wing must be able to carry whatever the rotors cannot.
//     L_wing_required = m*g - T_avail*cos(alpha)
//     q*S*CL_max/margin^2 >= L_wing_required
//  => cos(alpha) >= (m*g - q*S*CL_max/margin^2) / T_avail
// Only a fraction of max thrust may be assumed to go into lift - the rest is
// the reserve attitude control needs.
float FlightKinematics::corridorCeiling(float V, float rho) const {
    const float q          = 0.5f * rho * V * V;
    const float T_avail    = cfg_.corridor_thrust_fraction * 2.0f * cfg_.thrust_max_per_rotor;
    const float margin_sq  = cfg_.stall_margin * cfg_.stall_margin;
    const float lift_avail = q * cfg_.wing_area * cfg_.CL_max / margin_sq;
    const float rhs        = (cfg_.mass * cfg_.g - lift_avail) / fmaxf(T_avail, 1.0e-3f);
    if (rhs >= 1.0f)  return 0.0f;          // too slow to tilt at all
    if (rhs <= -1.0f) return kHalfPi;       // wing can carry everything
    return fminf(acosf(rhs), kHalfPi);
}

void FlightKinematics::corridor(float V, float rho, float &a_min, float &a_max) const {
    a_max = corridorCeiling(V, rho);

    // --- floor: do not fly fast with the nacelles near vertical.
    //     V <= v_max_hover + (v_max_fwd - v_max_hover)*sin(alpha)
    const float span = cfg_.v_max_forward - cfg_.v_max_hover;
    if (span <= 0.1f || V <= cfg_.v_max_hover) {
        a_min = 0.0f;
    } else {
        a_min = asinf(clampf((V - cfg_.v_max_hover) / span, 0.0f, 1.0f));
    }

    // Pinched (faster than the floor allows at the angle the wing can
    // support): lift wins - tilting further than the wing can carry means
    // losing altitude/stalling, while being a little too fast for the
    // nacelle angle only costs drag and rotor loads. The speed loop is
    // meanwhile slowing the aircraft back into the corridor.
    if (a_min > a_max) a_min = a_max;
}

float FlightKinematics::expectedDynamicPressure(float alpha, float thrust_total) const {
    // Level, unaccelerated equilibrium with induced drag included:
    //   T*sin(a) = q*S*CD0 + L_wing^2 / (q*S*pi*AR*e),  L_wing = m*g - T*cos(a)
    // => CD0*S*q^2 - T*sin(a)*q + L_wing^2/(S*pi*AR*e) = 0
    const float S      = cfg_.wing_area;
    const float L_wing = cfg_.mass * cfg_.g - thrust_total * cosf(alpha);
    const float a      = cfg_.CD0 * S;
    const float b      = -thrust_total * sinf(alpha);
    const float c      = (L_wing * L_wing) / (S * kPi * cfg_.aspect_ratio * cfg_.oswald_e);

    if (a <= 1.0e-9f) return -1.0f;
    const float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f) return -1.0f;  // no equilibrium: thrust cannot beat drag here

    // Take the high-speed root (front side of the drag curve).
    return (-b + sqrtf(disc)) / (2.0f * a);
}

float FlightKinematics::trimThrust(float alpha, float V, float rho) const {
    const float ca = cosf(alpha), sa = sinf(alpha);
    const float q  = 0.5f * rho * V * V;
    const float S  = cfg_.wing_area;
    if (sa < 0.02f) return cfg_.mass * cfg_.g / fmaxf(ca, 0.05f);

    // Fixed-point iteration: thrust affects wing loading affects induced drag.
    float T = cfg_.mass * cfg_.g;
    for (int i = 0; i < 6; ++i) {
        const float L_wing = cfg_.mass * cfg_.g - T * ca;
        float drag = q * S * cfg_.CD0;
        if (q > 1.0f) drag += (L_wing * L_wing) / (q * S * kPi * cfg_.aspect_ratio * cfg_.oswald_e);
        T = drag / sa;
        T = clampf(T, 0.0f, 2.0f * cfg_.thrust_max_per_rotor);
    }
    return T;
}

float FlightKinematics::hoverPitchAuthority(float thrust_total) const {
    // |dM / d(collective nacelle perturbation)| at alpha = 0.
    return thrust_total * cfg_.l_z;
}

float FlightKinematics::stallSpeed(float rho) const {
    const float denom = rho * cfg_.wing_area * cfg_.CL_max;
    if (!(denom > 0.0f)) return 1.0e6f;
    return sqrtf(2.0f * cfg_.mass * cfg_.g / denom);
}

float FlightKinematics::wingLift(const VehicleState &st, float V, float q_dyn) const {
    if (q_dyn < 1.0f) return 0.0f;
    const float Vs  = fmaxf(V, 1.0f);
    const float fpa = atan2f(st.climb_rate, Vs);       // flight path angle
    const float aoa = clampf(st.pitch - fpa, -0.35f, 0.35f);
    const float CL  = clampf(cfg_.CL_0 + cfg_.CL_alpha * aoa, -cfg_.CL_max, cfg_.CL_max);
    return q_dyn * cfg_.wing_area * CL;
}

// ===========================================================================
// mode machine  (hysteresis is mandatory - a single distance threshold chatters)
// ===========================================================================
void FlightKinematics::updateMode(const VehicleState &st) {
    if (!armed_)   { mode_ = FlightMode::DISARMED; alpha_target_ = alpha_cmd_; return; }
    if (failsafe_) { mode_ = FlightMode::FAILSAFE; alpha_target_ = 0.0f;       return; }

    // Conversion is a mission-ENROUTE-only affair, needs a live pitot (the
    // corridor and every forward-flight law depend on real airspeed), and is
    // judged on the distance left in the whole MISSION, not the segment.
    const bool enroute    = (phase_ == MissionPhase::ENROUTE);
    const bool far        = enroute && st.nav_valid && st.airspeed_valid &&
                            (st.mission_distance_to_go > cfg_.dist_to_forward);
    const bool need_hover = !enroute || !st.nav_valid || !st.airspeed_valid ||
                            (st.mission_distance_to_go < cfg_.dist_to_hover);

    switch (mode_) {
        case FlightMode::DISARMED:
        case FlightMode::FAILSAFE:
            mode_ = FlightMode::HOVER;
            break;

        case FlightMode::HOVER:
            if (far) mode_ = FlightMode::TRANS_FWD;
            break;

        case FlightMode::TRANS_FWD:
            if (need_hover)                         mode_ = FlightMode::TRANS_BACK;
            else if (alpha_cmd_ > kHalfPi - 0.03f)  mode_ = FlightMode::FORWARD;
            break;

        case FlightMode::FORWARD:
            if (need_hover) mode_ = FlightMode::TRANS_BACK;
            break;

        case FlightMode::TRANS_BACK:
            if (far)                     mode_ = FlightMode::TRANS_FWD;
            else if (alpha_cmd_ < 0.03f) mode_ = FlightMode::HOVER;
            break;
    }

    switch (mode_) {
        case FlightMode::TRANS_FWD:
        case FlightMode::FORWARD:    alpha_target_ = kHalfPi; break;
        default:                     alpha_target_ = 0.0f;    break;
    }
}

// ===========================================================================
// nacelle schedule: slew toward the target, clamped into the conversion corridor
// ===========================================================================
float FlightKinematics::scheduleAlpha(float dt, const VehicleState &st, ActuatorCmd &out) {
    // Speed reference for the corridor. The pitot is the only source that may
    // justify tilting TOWARD airplane mode. GNSS ground speed is a fallback
    // for moving back toward hover only: it overstates airspeed in a
    // tailwind (unsafe for the ceiling) but that only matters when tilting
    // forward, which the fallback never allows.
    float V = -1.0f;
    if (st.airspeed_valid)        V = st.airspeed;
    else if (st.ground_vel_valid) V = st.ground_speed;

    float want = alpha_target_;
    if (!st.airspeed_valid) want = fminf(want, alpha_cmd_);

    const float step = cfg_.alpha_rate_nominal * dt;
    float a_new = alpha_cmd_;
    if (want > alpha_cmd_)      a_new = fminf(alpha_cmd_ + step, want);
    else if (want < alpha_cmd_) a_new = fmaxf(alpha_cmd_ - step, want);

    if (V < 0.0f) {
        // No speed reference at all: cannot corridor-protect. Only ever move
        // toward hover, at the nominal rate - a tiltrotor stuck converted
        // can't land.
        out.alpha_min = 0.0f;
        out.alpha_max = alpha_cmd_;
        out.corridor_limited = true;
        alpha_cmd_ = clampf(a_new, 0.0f, kHalfPi);
        return alpha_cmd_;
    }

    float a_min, a_max;
    corridor(V, st.rho, a_min, a_max);

    float a_clamped = clampf(a_new, a_min, a_max);
    if (!st.airspeed_valid) a_clamped = fminf(a_clamped, alpha_cmd_);   // floor can't push it forward either
    out.corridor_limited  = (fabsf(a_clamped - a_new) > 1.0e-4f);
    out.alpha_min         = a_min;
    out.alpha_max         = a_max;

    alpha_cmd_ = clampf(a_clamped, 0.0f, kHalfPi);
    return alpha_cmd_;
}

// ===========================================================================
// effectiveness matrix: dv/du, linearised about (alpha, T_total, q)
// ===========================================================================
void FlightKinematics::buildEffectiveness(float alpha, float T, float q_dyn,
                                          float B[NV][NU]) const {
    for (int i = 0; i < NV; ++i)
        for (int k = 0; k < NU; ++k) B[i][k] = 0.0f;

    const float ca = cosf(alpha);
    const float sa = sinf(alpha);
    const float qS = q_dyn * cfg_.wing_area;
    const float b_ = cfg_.wing_span;
    const float c_ = cfg_.mean_chord;

    // Each nacelle carries half the thrust (linearised about dT = 0):
    // d/d(alpha_L) of T_L*cos(alpha_L) = -(T/2)*sin(alpha), etc.
    const float Th = 0.5f * T;

    // F_up = sum T_i*cos(alpha_i)
    B[V_FUP][U_THRUST] =  ca;
    B[V_FUP][U_NAC_L]  = -Th * sa;
    B[V_FUP][U_NAC_R]  = -Th * sa;

    // F_x = sum T_i*sin(alpha_i)
    B[V_FX][U_THRUST] = sa;
    B[V_FX][U_NAC_L]  = Th * ca;
    B[V_FX][U_NAC_R]  = Th * ca;

    // L = l_y*( T_L*cos(a_L) - T_R*cos(a_R) )
    B[V_L][U_DTHRUST] =  cfg_.l_y * ca;
    B[V_L][U_NAC_L]   = -cfg_.l_y * Th * sa;
    B[V_L][U_NAC_R]   =  cfg_.l_y * Th * sa;
    B[V_L][U_AILERON] =  qS * b_ * cfg_.Cl_da;

    // M = -l_z*sum T_i*sin(a_i) + l_x*sum T_i*cos(a_i)
    B[V_M][U_THRUST]   = -cfg_.l_z * sa + cfg_.l_x * ca;
    B[V_M][U_NAC_L]    = -Th * (cfg_.l_z * ca + cfg_.l_x * sa);
    B[V_M][U_NAC_R]    = -Th * (cfg_.l_z * ca + cfg_.l_x * sa);
    B[V_M][U_ELEVATOR] =  qS * c_ * cfg_.Cm_de;

    // N = l_y*( T_L*sin(a_L) - T_R*sin(a_R) )
    B[V_N][U_DTHRUST] =  cfg_.l_y * sa;
    B[V_N][U_NAC_L]   =  cfg_.l_y * Th * ca;
    B[V_N][U_NAC_R]   = -cfg_.l_y * Th * ca;
    B[V_N][U_RUDDER]  =  qS * b_ * cfg_.Cn_dr;

    // Note what happens automatically here: every aero column carries a factor
    // of q. At low airspeed those columns vanish and the allocator simply does
    // not use the surfaces; as q builds they take over. That is the blend you
    // were trying to hand-write with cos^2/sin^2, derived from physics instead.
}

// ===========================================================================
// weighted damped least squares with one saturation-redistribution pass
//   minimise ||W^(1/2) u||^2  subject to  (pri .* B) u ~= (pri .* v)
//   u = Winv*B'^T * (B'*Winv*B'^T + lambda*I)^-1 * v'
// ===========================================================================
void FlightKinematics::allocate(const float B[NV][NU], const float v[NV],
                                const float pri[NV], float alpha, float u[NU]) {
    const float w[NU] = {
        cfg_.w_thrust, cfg_.w_dthrust, cfg_.w_nacelle, cfg_.w_nacelle,
        cfg_.w_aileron, cfg_.w_elevator, cfg_.w_rudder
    };
    float winv[NU];
    for (int k = 0; k < NU; ++k) winv[k] = 1.0f / fmaxf(w[k], 1.0e-9f);

    // Row-scaled system (priority weighting).
    float Bp[NV][NU], vp[NV];
    for (int i = 0; i < NV; ++i) {
        vp[i] = pri[i] * v[i];
        for (int k = 0; k < NU; ++k) Bp[i][k] = pri[i] * B[i][k];
    }

    // Effector saturation limits.
    const float T_tot_min = 2.0f * cfg_.thrust_min_per_rotor;
    const float T_tot_max = 2.0f * cfg_.thrust_max_per_rotor;
    const float dT_max    = cfg_.thrust_max_per_rotor - cfg_.thrust_min_per_rotor;
    // Each nacelle: its control band around the schedule, cut to what the
    // mechanism can actually reach from here (e.g. at alpha = 0 it can go
    // 15 deg aft; at alpha = 90 deg only 5.7 deg further forward).
    const float nac_lo = fmaxf(-cfg_.nacelle_ctrl_band, cfg_.nacelle_min_rad - alpha);
    const float nac_hi = fminf( cfg_.nacelle_ctrl_band, cfg_.nacelle_max_rad - alpha);
    const float lo[NU] = { T_tot_min, -dT_max, nac_lo, nac_lo, -1.0f, -1.0f, -1.0f };
    const float hi[NU] = { T_tot_max,  dT_max, nac_hi, nac_hi,  1.0f,  1.0f,  1.0f };

    bool  free_[NU];
    for (int k = 0; k < NU; ++k) { u[k] = 0.0f; free_[k] = true; }

    for (int pass = 0; pass < 2; ++pass) {
        // residual objective
        float res[NV];
        for (int i = 0; i < NV; ++i) {
            float s = vp[i];
            for (int k = 0; k < NU; ++k) s -= Bp[i][k] * u[k];
            res[i] = s;
        }

        // A = Bp * Winv * Bp^T over free columns only, + relative damping
        float A[NV][NV];
        for (int i = 0; i < NV; ++i) {
            for (int j = 0; j <= i; ++j) {
                float s = 0.0f;
                for (int k = 0; k < NU; ++k)
                    if (free_[k]) s += Bp[i][k] * winv[k] * Bp[j][k];
                A[i][j] = s;
                A[j][i] = s;
            }
        }
        for (int i = 0; i < NV; ++i)
            A[i][i] += cfg_.lambda_reg * A[i][i] + 1.0e-6f;

        float L[NV][NV], x[NV];
        if (!cholDecompose(A, L)) return;   // degenerate: keep what we have
        cholSolve(L, res, x);

        // du = Winv * Bp^T * x   (free columns only)
        bool newly_saturated = false;
        for (int k = 0; k < NU; ++k) {
            if (!free_[k]) continue;
            float s = 0.0f;
            for (int i = 0; i < NV; ++i) s += Bp[i][k] * x[i];
            float cand = u[k] + winv[k] * s;
            if (cand > hi[k]) { cand = hi[k]; free_[k] = false; newly_saturated = true; }
            if (cand < lo[k]) { cand = lo[k]; free_[k] = false; newly_saturated = true; }
            u[k] = cand;
        }
        if (!newly_saturated) break;        // converged, no redistribution needed
    }
}

// ===========================================================================
// main update
// ===========================================================================
void FlightKinematics::update(float dt, const VehicleState &st, const Setpoints &sp,
                              ActuatorCmd &out) {
    // --- sanitise dt: a dropped cycle must not detonate the D or I terms ---
    if (!(dt > 1.0e-4f)) dt = 1.0e-4f;
    if (dt > 0.05f)      dt = 0.05f;

    updateMode(st);

    out.armed     = armed_;
    out.phase     = phase_;
    out.fs_reason = fs_reason_;

    if (mode_ == FlightMode::DISARMED) {
        resetControllers();
        out = ActuatorCmd{};
        out.armed     = false;
        out.phase     = phase_;
        out.fs_reason = fs_reason_;
        out.mode      = mode_;
        out.alpha_cmd = alpha_cmd_;
        out.nacelle_left  = nac_left_prev_;
        out.nacelle_right = nac_right_prev_;
        return;
    }

    // ---------------- air data ----------------
    // Pitot when valid. If it dies while converted (alpha > 0), fall back to
    // GNSS ground speed so the aero columns, wing-lift estimate and forward
    // speed loop don't collapse to zero mid-air - the mode machine is already
    // taking the aircraft back to hover. In hover w_fwd is ~0 regardless.
    float V     = 0.0f;
    float q_dyn = 0.0f;
    if (st.airspeed_valid) {
        V     = fmaxf(st.airspeed, 0.0f);
        q_dyn = fmaxf(st.q_dyn, 0.0f);
    } else if (st.ground_vel_valid && alpha_cmd_ > 0.05f) {
        V     = st.ground_speed;
        q_dyn = 0.5f * st.rho * V * V;
        out.air_data_synthetic = true;
    }

    // mu: how much authority the control surfaces actually have.
    const float mu = clampf(q_dyn / fmaxf(cfg_.q_ref, 1.0f), 0.0f, 1.0f);

    // ---------------- nacelle schedule ----------------
    const float alpha = scheduleAlpha(dt, st, out);
    const float sa    = sinf(alpha);

    // w_fwd arbitrates between the two OUTER-loop strategies (helicopter:
    // thrust holds altitude; airplane: pitch holds altitude, thrust holds
    // speed). This is a genuine soft handover between two controllers, unlike
    // the allocation below which is derived from the Jacobian.
    const float w_fwd = clampf(mu * sa, 0.0f, 1.0f);

    // ---------------- vertical / longitudinal outer loops ----------------
    const float az_dem_hover   = pid_climb_hover_.update(sp.climb_rate_sp, st.climb_rate, dt);
    const float pitch_frm_clmb = pid_climb_fwd_.update(sp.climb_rate_sp, st.climb_rate, dt);
    const float ax_dem_fwd     = pid_speed_fwd_.update(sp.airspeed_sp, V, dt);

    // ---------------- guidance cascade (path_active only) ----------------
    // Nose points along desired_course in every mode - lateral correction in
    // hover is via roll tilt, not by changing heading (no aim-off there).
    // Forward mode adds a cross-track-derived aim-off angle on top of
    // desired_course, single-stage. Hover gets a genuine two-stage cascade
    // (position error -> velocity setpoint -> tilt) since hover has little
    // natural damping to lean on.
    float heading_sp           = sp.desired_course;
    float roll_hover_guidance  = 0.0f;
    float pitch_hover_guidance = 0.0f;

    if (sp.path_active) {
        const float course_correction = clampf(
            pid_xtrack_to_course_fwd_.update(0.0f, sp.cross_track_error, dt),
            -cfg_.max_course_correction, cfg_.max_course_correction);
        heading_sp = sp.desired_course + w_fwd * course_correction;

        // Path-frame ground velocity from the GNSS velocity VECTOR. Heading
        // can't stand in for it: heading hold keeps the nose on the course, so
        // sin(yaw - course) reads ~0 whatever the real sideways drift is.
        float lateral_vel = 0.0f, along_vel = 0.0f;
        if (st.ground_vel_valid) {
            const float rel = st.course_over_ground - sp.desired_course;
            lateral_vel = st.ground_speed * sinf(rel);   // right of path positive
            along_vel   = st.ground_speed * cosf(rel);
        }
        const float lateral_vel_sp = pid_xtrack_to_latvel_hover_.update(0.0f, sp.cross_track_error, dt);

        roll_hover_guidance = clampf(
            pid_latvel_to_roll_hover_.update(lateral_vel_sp, lateral_vel, dt),
            -cfg_.lateral_hold_max_tilt, cfg_.lateral_hold_max_tilt);

        // NOTE the negation: to accelerate FORWARD, the fuselage must pitch
        // NOSE-DOWN (negative, in this file's nose-up-positive convention) -
        // that's what tilts the mostly-vertical hover thrust vector forward.
        // Nacelle tilt isn't available as a separate forward-thrust channel
        // here since alpha is locked at 0 in HOVER (w_fwd suppresses F_x
        // until real forward flight) - pitch-based translation, same
        // mechanism a multicopter uses, is the only channel HOVER mode has.
        pitch_hover_guidance = -clampf(
            pid_speed_hover_.update(sp.along_speed_sp, along_vel, dt),
            -cfg_.lateral_hold_max_tilt, cfg_.lateral_hold_max_tilt);

        out.cross_track_dbg     = sp.cross_track_error;
        out.lateral_vel_est_dbg = lateral_vel;
        out.lateral_vel_sp_dbg  = lateral_vel_sp;
        out.along_vel_est_dbg   = along_vel;
        out.along_vel_sp_dbg    = sp.along_speed_sp;
    } else {
        pid_xtrack_to_course_fwd_.reset();
        pid_xtrack_to_latvel_hover_.reset();
        pid_latvel_to_roll_hover_.reset();
        pid_speed_hover_.reset();
    }

    // ---------------- attitude demands ----------------
    // The stabiliser ALWAYS regulates roll and pitch. Outside path-following,
    // the config flags decide whether guidance/RC may command a non-zero
    // hover attitude directly - "unused DOF" means "not commanded", never
    // "not held".
    const float lim = cfg_.hover_attitude_limit;
    const float roll_hover  = sp.path_active ? roll_hover_guidance
                              : (cfg_.hover_allow_roll_cmd ? clampf(sp.roll_sp, -lim, lim) : 0.0f);
    const float pitch_hover = sp.path_active ? pitch_hover_guidance
                              : (cfg_.hover_allow_pitch_cmd ? clampf(sp.pitch_sp, -lim, lim) : 0.0f);

    // Heading error, wrapped to [-pi,pi] - a plain PID doesn't know about
    // angle wraparound, so the wrap happens here, not inside pid_heading_hold_.
    float yaw_rate_from_heading = sp.yaw_rate_sp;
    if (sp.path_active) {
        const float herr = wrapPi(heading_sp - st.yaw);
        yaw_rate_from_heading = pid_heading_hold_.update(herr, 0.0f, dt);
        out.heading_error_dbg = herr;
    } else {
        pid_heading_hold_.reset();
    }

    // Forward-mode bank command from the desired turn rate - the inverse of
    // the coordinated-turn relation psi_dot_fwd already uses below (that one
    // goes bank->rate for anti-sideslip; this one goes rate->bank to
    // actually steer toward heading_sp).
    const float roll_fwd_guidance = clampf(
        (V > 1.0f) ? atan2f(V * yaw_rate_from_heading, cfg_.g) : 0.0f,
        -cfg_.max_bank_fwd, cfg_.max_bank_fwd);

    const float roll_fwd  = sp.path_active ? roll_fwd_guidance : sp.roll_sp;
    const float pitch_fwd = clampf(sp.pitch_sp + pitch_frm_clmb, -0.45f, 0.45f);

    const float roll_cmd  = lerpf(roll_hover,  roll_fwd,  w_fwd);
    const float pitch_cmd = lerpf(pitch_hover, pitch_fwd, w_fwd);

    // Turn demand: direct in helicopter mode, bank-coordinated in airplane
    // mode. yaw_rate_from_heading already stands in for sp.yaw_rate_sp when
    // path_active (and equals it exactly otherwise).
    float psi_dot_fwd = yaw_rate_from_heading;
    if (V > 5.0f) psi_dot_fwd = (cfg_.g / V) * tanf(clampf(st.roll, -1.0f, 1.0f));
    const float psi_dot = lerpf(yaw_rate_from_heading, psi_dot_fwd, w_fwd);

    // ---------------- Euler kinematic transform ----------------
    // This is the correct, SIGNED replacement for cos^2/sin^2 bank blending.
    // At phi = 0 it is the identity; at phi = +/-90 deg elevator and rudder
    // swap roles WITH the right sign, which squaring cannot express.
    const float sphi = sinf(st.roll),  cphi = cosf(st.roll);
    const float sth  = sinf(st.pitch), cth  = cosf(st.pitch);

    const float phi_dot_des   = gains_.kp_roll_att  * (roll_cmd  - st.roll);
    const float theta_dot_des = gains_.kp_pitch_att * (pitch_cmd - st.pitch);

    const float p_sp =  phi_dot_des - sth * psi_dot;
    const float q_sp =  cphi * theta_dot_des + sphi * cth * psi_dot;
    const float r_sp = -sphi * theta_dot_des + cphi * cth * psi_dot;

    // ---------------- inner rate loops -> angular acceleration ----------------
    const float pdot = pid_rate_roll_.update(p_sp, st.p, dt);
    const float qdot = pid_rate_pitch_.update(q_sp, st.q, dt);
    const float rdot = pid_rate_yaw_.update(r_sp, st.r, dt);

    // ---------------- objective vector ----------------
    const float lift_w = wingLift(st, V, q_dyn);

    float drag_est = q_dyn * cfg_.wing_area * cfg_.CD0;
    if (q_dyn > 1.0f) {
        drag_est += (lift_w * lift_w) /
                    (q_dyn * cfg_.wing_area * kPi * cfg_.aspect_ratio * cfg_.oswald_e);
    }

    float v[NV];
    // Rotors only have to carry what the wing does not. As q builds through the
    // transition, lift_w rises and this demand falls out on its own.
    v[V_FUP] = fmaxf(cfg_.mass * (cfg_.g + (1.0f - w_fwd) * az_dem_hover) - lift_w, 0.0f);
    v[V_FX]  = w_fwd * (cfg_.mass * ax_dem_fwd + drag_est);
    v[V_L]   = cfg_.Ixx * pdot;
    v[V_M]   = cfg_.Iyy * qdot;
    v[V_N]   = cfg_.Izz * rdot;

    float pri[NV];
    pri[V_FUP] = cfg_.pri_Fup;
    pri[V_FX]  = lerpf(cfg_.pri_Fx_hover, cfg_.pri_Fx_fwd, w_fwd);
    pri[V_L]   = cfg_.pri_moment;
    pri[V_M]   = cfg_.pri_moment;
    pri[V_N]   = cfg_.pri_moment;

    // ---------------- allocation ----------------
    // Seed the Jacobian with last cycle's thrust; floor it so the matrix never
    // collapses to zero on the first iteration or after a throttle chop.
    const float T_est = fmaxf(thrust_total_, 0.3f * cfg_.mass * cfg_.g);

    float B[NV][NU], u[NU];
    buildEffectiveness(alpha, T_est, q_dyn, B);
    allocate(B, v, pri, alpha, u);

    // ---------------- map effectors to hardware ----------------
    const float T_total = clampf(u[U_THRUST],
                                 2.0f * cfg_.thrust_min_per_rotor,
                                 2.0f * cfg_.thrust_max_per_rotor);
    thrust_total_ = T_total;

    float T_L = 0.5f * (T_total + u[U_DTHRUST]);
    float T_R = 0.5f * (T_total - u[U_DTHRUST]);
    T_L = clampf(T_L, cfg_.thrust_min_per_rotor, cfg_.thrust_max_per_rotor);
    T_R = clampf(T_R, cfg_.thrust_min_per_rotor, cfg_.thrust_max_per_rotor);

    // Static thrust goes roughly as rpm^2 and rpm roughly as throttle, so
    // sqrt() linearises the command. Replace with a measured thrust curve once
    // you have static-test data - it is the single cheapest gain-scheduling win.
    out.thrust_left  = clampf(sqrtf(T_L / cfg_.thrust_max_per_rotor), 0.0f, 1.0f);
    out.thrust_right = clampf(sqrtf(T_R / cfg_.thrust_max_per_rotor), 0.0f, 1.0f);

    // The allocator already kept both inside the mechanical range; the clamp
    // only guards against a degenerate solve.
    float nac_L = clampf(alpha + u[U_NAC_L], cfg_.nacelle_min_rad, cfg_.nacelle_max_rad);
    float nac_R = clampf(alpha + u[U_NAC_R], cfg_.nacelle_min_rad, cfg_.nacelle_max_rad);

    // servo slew limit
    const float max_step = cfg_.nacelle_rate_max * dt;
    nac_L = clampf(nac_L, nac_left_prev_  - max_step, nac_left_prev_  + max_step);
    nac_R = clampf(nac_R, nac_right_prev_ - max_step, nac_right_prev_ + max_step);
    nac_left_prev_  = nac_L;
    nac_right_prev_ = nac_R;

    out.nacelle_left  = nac_L;
    out.nacelle_right = nac_R;
    out.aileron  = clampf(u[U_AILERON],  -1.0f, 1.0f);
    out.elevator = clampf(u[U_ELEVATOR], -1.0f, 1.0f);
    out.rudder   = clampf(u[U_RUDDER],   -1.0f, 1.0f);

    // ---------------- telemetry ----------------
    out.mode          = mode_;
    out.alpha_cmd     = alpha;
    out.mu            = mu;
    out.wing_lift_est = lift_w;
    out.thrust_total  = T_total;
    for (int i = 0; i < NV; ++i) out.v_demand[i]  = v[i];
    for (int k = 0; k < NU; ++k) out.u_applied[k] = u[k];
}

// ===========================================================================
// runCycle - the hardware-facing entry point. Everything above this point in
// the file is pure control-law math with no sensor-format opinions; this
// function is the one place that changes if your AHRS's raw convention ever
// changes. See the header comment block at the top of flight_kinematics.h
// for why these particular signs (roll unchanged, pitch/yaw negated).
// ===========================================================================
ActuatorCmd FlightKinematics::runCycle(float dt, const RawSensors &raw,
                                       const Setpoints &sp_manual, VehicleState *out_state) {
    // Estimators get the real elapsed time (clamped against nonsense);
    // update() applies its own tighter clamp for the PIDs.
    if (!(dt > 0.0f)) dt = 0.0f;
    if (dt > 0.5f)    dt = 0.5f;
    time_s_ += dt;

    VehicleState st;

    // --- attitude: Fusion's raw NWU -> this module's internal convention ---
    st.roll  =  raw.fusion_roll_deg  * kDegToRad;   // unchanged
    st.pitch = -raw.fusion_pitch_deg * kDegToRad;   // raw nose-down+ -> nose-up+
    st.yaw   = wrapPi(-raw.fusion_yaw_deg * kDegToRad   // raw nose-left+ -> nose-right+
                      + cfg_.magnetic_declination_rad); // magnetic -> true north
    st.p     =  raw.fusion_gyro_x_dps * kDegToRad;  // unchanged
    st.q     = -raw.fusion_gyro_y_dps * kDegToRad;
    st.r     = -raw.fusion_gyro_z_dps * kDegToRad;

    // --- horizontal: GNSS dead-reckoning (also feeds the GNSS climb-rate
    //     fallback below, so it runs first) ---
    updateNavEstimate(dt, raw, st);

    // --- vertical channel: baro/GNSS complementary filter ---
    // Baro's climb_rate drives fast/responsive tracking (P0-bias-immune -
    // differentiating cancels a near-constant offset). GNSS altitude slowly
    // calibrates OUT that bias via a separately-tracked estimate
    // (baro_bias_m_), rather than being applied to fused_altitude_m_
    // directly - see baro_resync_tau_s's header comment.
    if (raw.baro_valid) {
        if (!alt_fusion_primed_) {
            fused_altitude_m_  = raw.baro_altitude;
            baro_frame_alt_m_  = raw.baro_altitude;
            alt_fusion_primed_ = true;
        } else if (dt > 1.0e-4f) {
            fused_altitude_m_ += raw.baro_climb_rate * dt;
            baro_frame_alt_m_ += raw.baro_climb_rate * dt;

            // Fast re-sync toward the BIAS-CORRECTED raw reading (and, for
            // the baro-only copy, toward the raw reading itself).
            const float correctedBaro = raw.baro_altitude - baro_bias_m_;
            const float tauFast   = fmaxf(cfg_.baro_resync_tau_s, 0.1f);
            const float alphaFast = dt / (tauFast + dt);
            fused_altitude_m_ += alphaFast * (correctedBaro - fused_altitude_m_);
            baro_frame_alt_m_ += alphaFast * (raw.baro_altitude - baro_frame_alt_m_);

            if (raw.gnss_fix_valid && st.nav_valid) {
                // Adapt the BIAS estimate (not fused_altitude_m_ directly)
                // toward whatever offset currently reconciles raw baro with
                // GNSS truth - P0 miscalibration, temperature, weather.
                // Faster on the ground, where it's calibrating before takeoff.
                const float tauSlow   = fmaxf(armed_ ? cfg_.baro_gnss_fusion_tau_s
                                                     : cfg_.baro_gnss_fusion_tau_ground_s, 0.1f);
                const float alphaSlow = dt / (tauSlow + dt);
                const float impliedBias = raw.baro_altitude - raw.gnss_altitude_m;
                baro_bias_m_ += alphaSlow * (impliedBias - baro_bias_m_);
            }
        }
    }

    if (raw.baro_valid && raw.gnss_fix_valid && st.nav_valid)
        gnss_alt_settle_s_ = fminf(gnss_alt_settle_s_ + dt, 1.0e6f);
    else
        gnss_alt_settle_s_ = 0.0f;

    // Fallback AGL (no DEM): height above the arming point, measured on the
    // baro-only smoothed altitude against a reference captured on it at
    // arm(), so the GNSS bias estimate converging can't shift it. Overridden
    // below by the DEM clearance when available.
    if (raw.baro_valid && alt_fusion_primed_) {
        st.altitude   = fused_altitude_m_;
        st.climb_rate = raw.baro_climb_rate;
        st.agl        = ground_ref_set_ ? (baro_frame_alt_m_ - ground_ref_baro_m_) : 0.0f;
    } else if (raw.gnss_fix_valid && st.nav_valid) {
        // Barometer lost: GNSS-only vertical channel (noisier, laggier) -
        // enough for the failsafe descent that updateHealth() will trigger.
        st.altitude   = raw.gnss_altitude_m;
        st.climb_rate = gnss_climb_est_.rate();
        st.agl        = ground_ref_set_ ? (raw.gnss_altitude_m - (ground_ref_baro_m_ - baro_bias_m_)) : 0.0f;
    } else {
        st.altitude   = fused_altitude_m_;
        st.climb_rate = 0.0f;
        st.agl        = ground_ref_set_ ? (baro_frame_alt_m_ - ground_ref_baro_m_) : 0.0f;
    }

    // DEM clearance: MSL altitude minus the highest terrain under the vehicle
    // and along the look-ahead. Both are absolute (GNSS-calibrated MSL, DEM
    // MSL), so nothing depends on where the vehicle was armed.
    if (raw.terrain_valid) {
        st.terrain_valid  = true;
        st.terrain_elev_m = raw.terrain_elev_m;
        st.agl            = st.altitude - raw.terrain_elev_m;
    }

    // Air density from the ideal gas law using real pressure/temperature
    // instead of a fixed 1.225 - dynamic-pressure-derived airspeed and the
    // corridor both scale with it. 1000 Pa = 10 hPa is a sensor-alive sanity
    // floor, not a physical-altitude gate.
    if (raw.baro_valid && raw.baro_pressure_pa > 1000.0f) {
        const float T_kelvin = raw.baro_temperature_c + 273.15f;
        const float R_specific_air = 287.05f;  // J/(kg*K), dry air
        st.rho = raw.baro_pressure_pa / (R_specific_air * T_kelvin);
    } else {
        st.rho = 1.225f;  // ISA sea-level fallback if baro isn't reporting
    }

    // --- air data: pitot differential pressure -> true airspeed ---
    st.airspeed_valid = raw.pitot_valid;
    st.q_dyn          = raw.pitot_valid ? fmaxf(raw.pitot_q_pa, 0.0f) : 0.0f;
    st.airspeed       = raw.pitot_valid ? sqrtf(2.0f * st.q_dyn / st.rho) : 0.0f;

    // --- sensor health -> failsafe ---
    updateHealth(dt, raw);

    // --- guidance: failsafe > mission > manual ---
    Setpoints sp = sp_manual;
    if (armed_) {
        if (failsafe_)                          sp = runFailsafeGuidance(dt, st);
        else if (phase_ != MissionPhase::NONE)  sp = runMissionGuidance(dt, st);
        // runMissionGuidance may itself latch a failsafe (NAV_LOST); it takes
        // effect next cycle.
    }

    ActuatorCmd out;
    update(dt, st, sp, out);

    // --- touchdown detection (only while a landing descent is commanded) ---
    // On the ground the climb-rate loop keeps asking for -descent_rate, sees
    // ~0, and winds thrust down - so "not moving vertically AND asking for
    // well under hover thrust" only happens after touchdown. Except right at
    // the start of the descent, before the slow baro climb-rate estimate has
    // caught up - hence land_detect_delay_s.
    if (armed_ && sp.descending) {
        descend_time_s_ += dt;
        const bool still      = fabsf(st.climb_rate) < 0.3f;
        const bool low_thrust = out.v_demand[V_FUP] < 0.7f * cfg_.mass * cfg_.g;
        const bool armed_det  = descend_time_s_ > cfg_.land_detect_delay_s;
        landed_timer_s_ = (armed_det && still && low_thrust) ? landed_timer_s_ + dt : 0.0f;
        if (landed_timer_s_ >= cfg_.landed_confirm_s) {
            disarm();
            update(dt, st, sp, out);   // emit disarmed outputs this very cycle
        }
    } else {
        landed_timer_s_ = 0.0f;
        descend_time_s_ = 0.0f;
    }

    last_raw_  = raw;
    last_st_   = st;
    have_last_ = true;

    if (out_state) *out_state = st;
    return out;
}

// ===========================================================================
// updateNavEstimate - GNSS position, dead-reckoned between fixes.
//
// GNSS only reports a genuinely NEW fix at ~1 Hz (gnss_fix_seq changes), but
// this runs every 100 Hz cycle. The estimate is extrapolated every cycle
// with the GNSS velocity VECTOR (speed + course over ground from RMC) - not
// the nose heading, which differs from the direction of travel whenever the
// aircraft crabs into wind or drifts in hover, and is magnetic - then pulled
// toward each fresh fix using the ACTUAL interval since the previous one.
// ===========================================================================
void FlightKinematics::updateNavEstimate(float dt, const RawSensors &raw, VehicleState &st) {
    // --- velocity (RMC) ---
    const bool newVel = raw.gnss_vel_valid && (raw.gnss_vel_seq != last_vel_seq_seen_);
    if (newVel) {
        last_vel_seq_seen_ = raw.gnss_vel_seq;
        time_since_vel_s_  = 0.0f;
        gnss_speed_        = raw.gnss_ground_speed;
        gnss_course_       = raw.gnss_course_rad;
        gnss_vel_have_     = true;
    } else {
        time_since_vel_s_ += dt;
    }
    const bool vel_ok = gnss_vel_have_ && (time_since_vel_s_ < cfg_.gnss_fix_stale_timeout_s);
    st.ground_vel_valid   = vel_ok;
    st.ground_speed       = vel_ok ? gnss_speed_ : 0.0f;
    st.course_over_ground = gnss_course_;
    const float vx = vel_ok ? gnss_speed_ * sinf(gnss_course_) : 0.0f;   // east
    const float vy = vel_ok ? gnss_speed_ * cosf(gnss_course_) : 0.0f;   // north

    // --- position (GGA) ---
    const bool  newFix       = raw.gnss_fix_valid && (raw.gnss_fix_seq != last_gnss_seq_seen_);
    const float fix_interval = time_since_fix_s_;   // gap since the PREVIOUS new fix
    if (newFix) {
        last_gnss_seq_seen_ = raw.gnss_fix_seq;
        time_since_fix_s_   = 0.0f;
        // Vertical fallback, sampled at the fix rate (see RateEstimator).
        gnss_climb_est_.update(raw.gnss_altitude_m, fmaxf(fix_interval, 0.01f));
        // No mission: anchor the local frame at the first fix so positions
        // stay small (float precision) for failsafe position hold.
        if (!fcode::hasOrigin()) fcode::setOrigin(raw.gnss_lat_deg, raw.gnss_lon_deg);
    } else {
        time_since_fix_s_ += dt;
    }

    if (!pos_est_primed_) {
        if (newFix) {
            fcode::gpsToLocal(raw.gnss_lat_deg, raw.gnss_lon_deg, pos_est_x_m_, pos_est_y_m_);
            pos_est_primed_ = true;
        }
    } else {
        pos_est_x_m_ += vx * dt;
        pos_est_y_m_ += vy * dt;

        if (newFix) {
            float rawX, rawY;
            fcode::gpsToLocal(raw.gnss_lat_deg, raw.gnss_lon_deg, rawX, rawY);
            // Bounded pull toward the fresh fix, not a snap - a single noisy
            // fix shouldn't jerk the estimate.
            const float tau   = fmaxf(cfg_.gnss_position_correction_tau_s, 0.01f);
            const float alpha = 1.0f - expf(-fmaxf(fix_interval, 0.01f) / tau);
            pos_est_x_m_ += alpha * (rawX - pos_est_x_m_);
            pos_est_y_m_ += alpha * (rawY - pos_est_y_m_);
        }
    }

    if (time_since_fix_s_ > cfg_.gnss_fix_stale_timeout_s) {
        // GPS genuinely unavailable (not just between normal ~1Hz updates) -
        // stop trusting the dead-reckoned estimate; re-prime cleanly on the
        // next good fix rather than quietly drifting further on stale data.
        pos_est_primed_ = false;
    }

    st.nav_valid = pos_est_primed_;
    st.pos_x     = pos_est_x_m_;
    st.pos_y     = pos_est_y_m_;
}

// ===========================================================================
// updateHealth - sensor loss while armed -> latched FAILSAFE (land).
// ===========================================================================
void FlightKinematics::updateHealth(float dt, const RawSensors &raw) {
    if (!armed_) {
        imu_bad_s_  = 0.0f;
        baro_bad_s_ = 0.0f;
        return;
    }
    imu_bad_s_  = raw.imu_valid  ? 0.0f : imu_bad_s_  + dt;
    baro_bad_s_ = raw.baro_valid ? 0.0f : baro_bad_s_ + dt;
    if (imu_bad_s_  > cfg_.imu_fail_timeout_s)  requestFailsafe(FailsafeReason::IMU);
    if (baro_bad_s_ > cfg_.baro_fail_timeout_s) requestFailsafe(FailsafeReason::BARO);
}

// ===========================================================================
// guidance helpers
// ===========================================================================
float FlightKinematics::clearanceHold(float target_agl, const VehicleState &st, float dt) {
    last_target_agl_ = target_agl;
    if (st.terrain_valid) {
        if (alt_ref_msl_) { pid_altitude_hold_.reset(); alt_ref_msl_ = false; }
        terrain_lost_s_ = 0.0f;
        return pid_altitude_hold_.update(target_agl, st.agl, dt);
    }
    // No DEM for this position: hold the MSL altitude we had. The PID is
    // reset on the switch - its derivative-on-measurement would otherwise
    // see the measurement jump from ~clearance to ~MSL and kick.
    if (!alt_ref_msl_) {
        pid_altitude_hold_.reset();
        msl_hold_m_  = st.altitude;
        alt_ref_msl_ = true;
    }
    if (st.nav_valid) {   // with GNSS lost as well, NAV_LOST governs instead
        terrain_lost_s_ += dt;
        if (terrain_lost_s_ > cfg_.terrain_loss_land_timeout_s) requestFailsafe(FailsafeReason::TERRAIN);
    }
    return pid_altitude_hold_.update(msl_hold_m_, st.altitude, dt);
}

// Hold a point: path frame is the line through (hold_x, hold_y) along
// `course`, so the same hover cascade that follows paths also parks the
// aircraft - cross-track -> roll, signed along-track error -> velocity -> pitch.
void FlightKinematics::pointHold(const VehicleState &st, float hold_x, float hold_y,
                                 float course, Setpoints &sp) const {
    const float ux = sinf(course), uy = cosf(course);
    const float along_to_go = ux * (hold_x - st.pos_x) + uy * (hold_y - st.pos_y);
    const float cross       = uy * (st.pos_x - hold_x) - ux * (st.pos_y - hold_y);
    sp.path_active       = true;
    sp.desired_course    = course;
    sp.cross_track_error = cross;
    sp.along_speed_sp    = clampf(cfg_.point_hold_gain * along_to_go,
                                  -cfg_.point_hold_max_speed, cfg_.point_hold_max_speed);
}

// ===========================================================================
// runMissionGuidance - mission lifecycle TAKEOFF -> ENROUTE -> LAND, plus the
// altitude hold that turns F43's ground_clearance_m into climb_rate_sp.
// Fills st.distance_to_go / st.mission_distance_to_go for the mode machine.
// ===========================================================================
Setpoints FlightKinematics::runMissionGuidance(float dt, VehicleState &st) {
    Setpoints sp{};   // path_active false: wings level, hold heading - the
                      // safe default if anything below bails early
    sp.airspeed_sp = (mode_ == FlightMode::TRANS_BACK) ? cfg_.v_max_hover : cfg_.cruise_airspeed;

    if (!st.nav_valid) {
        // GNSS lost: the mode machine is already heading back to hover. Hold
        // wings level and altitude, and wait for the fix to come back - land
        // if it doesn't.
        nav_lost_s_ += dt;
        if (nav_lost_s_ > cfg_.nav_loss_land_timeout_s) requestFailsafe(FailsafeReason::NAV_LOST);
        if (phase_ == MissionPhase::LAND && land_descending_) {
            sp.climb_rate_sp = -cfg_.land_descent_rate;
            sp.descending    = true;
        } else {
            sp.climb_rate_sp = clearanceHold(last_target_agl_, st, dt);
        }
        return sp;
    }
    nav_lost_s_ = 0.0f;

    fcode::NavOutput nav;
    fcode::update(st.pos_x, st.pos_y, nav);
    st.distance_to_go         = nav.distance_to_go;
    st.mission_distance_to_go = nav.mission_distance_to_go;
    const float clearance     = nav.ground_clearance_m;

    switch (phase_) {
        case MissionPhase::TAKEOFF:
            // Vertical climb over the arming point, nose along the first leg -
            // don't start translating low over the ground.
            if (!takeoff_hold_set_) {
                hold_x_ = st.pos_x;
                hold_y_ = st.pos_y;
                hold_course_ = nav.desired_course_rad;
                takeoff_hold_set_ = true;
            }
            pointHold(st, hold_x_, hold_y_, hold_course_, sp);
            sp.climb_rate_sp = clearanceHold(clearance, st, dt);
            if (st.terrain_valid && st.agl >= cfg_.takeoff_complete_fraction * clearance)
                phase_ = MissionPhase::ENROUTE;
            break;

        case MissionPhase::ENROUTE:
            sp.path_active       = true;
            sp.desired_course    = nav.desired_course_rad;
            sp.cross_track_error = nav.cross_track_error;
            // Cruise, tapering as sqrt(2*a*d) into the FINAL point only -
            // intermediate segment ends are flown through.
            sp.along_speed_sp = fminf(cfg_.cruise_ground_speed_hover,
                                      sqrtf(2.0f * cfg_.approach_decel *
                                            fmaxf(nav.mission_distance_to_go, 0.0f)));
            sp.climb_rate_sp = clearanceHold(clearance, st, dt);
            if (nav.mission_complete) {
                phase_           = MissionPhase::LAND;
                land_descending_ = false;
                land_timer_s_    = 0.0f;
            }
            break;

        case MissionPhase::LAND: {
            // Hold over the final point; descend once back in hover and
            // settled (or after a timeout, e.g. in gusty wind).
            sp.path_active       = true;
            sp.desired_course    = nav.desired_course_rad;
            sp.cross_track_error = nav.cross_track_error;
            sp.along_speed_sp    = clampf(cfg_.point_hold_gain * nav.mission_distance_to_go,
                                          -cfg_.point_hold_max_speed, cfg_.point_hold_max_speed);
            land_timer_s_ += dt;
            if (!land_descending_) {
                const bool settled = (mode_ == FlightMode::HOVER) && (alpha_cmd_ < 0.03f) &&
                                     fabsf(nav.mission_distance_to_go) < cfg_.land_settle_radius_m &&
                                     fabsf(nav.cross_track_error)      < cfg_.land_settle_radius_m &&
                                     st.ground_speed < 1.0f;
                if (settled || land_timer_s_ > cfg_.land_settle_timeout_s) land_descending_ = true;
            }
            if (land_descending_) {
                sp.climb_rate_sp = -cfg_.land_descent_rate;
                sp.descending    = true;
            } else {
                sp.climb_rate_sp = clearanceHold(clearance, st, dt);
            }
            break;
        }

        case MissionPhase::NONE:
            break;
    }

    return sp;
}

// ===========================================================================
// runFailsafeGuidance - controlled landing where we are. Converting back to
// hover is the mode machine's job (FAILSAFE targets alpha = 0); this only
// decides attitude and the vertical channel.
// ===========================================================================
Setpoints FlightKinematics::runFailsafeGuidance(float dt, const VehicleState &st) {
    (void)dt;
    Setpoints sp{};                        // wings level, zero yaw rate
    sp.airspeed_sp = cfg_.v_max_hover;     // decelerate if still converted

    const bool converted_back = (alpha_cmd_ < 0.03f);
    if (!converted_back) {
        sp.climb_rate_sp = 0.0f;           // hold altitude while converting
        return sp;
    }

    if (st.nav_valid) {
        if (!fs_hold_set_) {
            hold_x_      = st.pos_x;
            hold_y_      = st.pos_y;
            hold_course_ = st.yaw;
            fs_hold_set_ = true;
        }
        pointHold(st, hold_x_, hold_y_, hold_course_, sp);
    }
    sp.climb_rate_sp = -cfg_.land_descent_rate;
    sp.descending    = true;
    return sp;
}

}  // namespace raven
