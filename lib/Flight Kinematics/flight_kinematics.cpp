// flight_kinematics.cpp - see flight_kinematics.h for conventions and the
// derivation of the effectiveness matrix.

#include "flight_kinematics.h"

#include <math.h>

namespace raven {

// ---------------------------------------------------------------------------
// small float helpers
// ---------------------------------------------------------------------------
static const float kPi     = 3.14159265f;
static const float kHalfPi = 1.57079633f;

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

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

// Effector indices into u[7]
enum : int {
    U_THRUST = 0,   // N, total thrust  (T_L + T_R)
    U_DTHRUST,      // N, differential  (T_L - T_R)
    U_DA_PITCH,     // rad, collective nacelle perturbation about the schedule
    U_DA_YAW,       // rad, differential nacelle tilt
    U_AILERON,      // -1..1
    U_ELEVATOR,     // -1..1
    U_RUDDER,       // -1..1
    NU = 7
};

// Objective indices into v[5]
enum : int { V_FUP = 0, V_FX, V_L, V_M, V_N, NV = 5 };

// ---------------------------------------------------------------------------
// Cholesky solve for the 5x5 symmetric positive-definite normal equations.
// ~100 flops, single precision, no allocation. Trivial at 400 Hz on an ESP32.
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
// setup
// ===========================================================================
void FlightKinematics::begin(const VehicleConfig &cfg, const ControlGains &gains) {
    cfg_   = cfg;
    gains_ = gains;

    pid_rate_roll_.setGains(gains_.rate_roll);
    pid_rate_pitch_.setGains(gains_.rate_pitch);
    pid_rate_yaw_.setGains(gains_.rate_yaw);
    pid_climb_hover_.setGains(gains_.climb_hover);
    pid_climb_fwd_.setGains(gains_.climb_fwd);
    pid_speed_fwd_.setGains(gains_.speed_fwd);

    reset();
}

void FlightKinematics::reset() {
    pid_rate_roll_.reset();
    pid_rate_pitch_.reset();
    pid_rate_yaw_.reset();
    pid_climb_hover_.reset();
    pid_climb_fwd_.reset();
    pid_speed_fwd_.reset();

    mode_           = armed_ ? FlightMode::HOVER : FlightMode::DISARMED;
    alpha_cmd_      = 0.0f;
    alpha_target_   = 0.0f;
    thrust_total_   = cfg_.mass * cfg_.g;
    nac_left_prev_  = 0.0f;
    nac_right_prev_ = 0.0f;
    failsafe_       = false;
}

void FlightKinematics::setArmed(bool armed) {
    if (armed && !armed_) {
        reset();
        armed_ = true;
        mode_  = FlightMode::HOVER;
    } else if (!armed) {
        armed_ = false;
        mode_  = FlightMode::DISARMED;
    }
}

void FlightKinematics::requestFailsafe() {
    failsafe_ = true;
    mode_     = FlightMode::FAILSAFE;
}

// ===========================================================================
// analysis helpers
// ===========================================================================
void FlightKinematics::corridor(float V, float rho, float &a_min, float &a_max) const {
    const float q          = 0.5f * rho * V * V;
    const float T_max_tot  = 2.0f * cfg_.thrust_max_per_rotor;
    const float margin_sq  = cfg_.stall_margin * cfg_.stall_margin;

    // --- ceiling: the wing must be able to carry whatever the rotors cannot.
    //     L_wing_required = m*g - T_max_total*cos(alpha)
    //     q*S*CL_max/margin^2 >= L_wing_required
    //  => cos(alpha) >= (m*g - q*S*CL_max/margin^2) / T_max_total
    const float lift_avail = q * cfg_.wing_area * cfg_.CL_max / margin_sq;
    const float rhs        = (cfg_.mass * cfg_.g - lift_avail) / T_max_tot;
    if (rhs >= 1.0f) {
        a_max = 0.0f;                       // too slow to tilt at all
    } else if (rhs <= -1.0f) {
        a_max = kHalfPi;                    // wing can carry everything
    } else {
        a_max = acosf(rhs);
        if (a_max > kHalfPi) a_max = kHalfPi;
    }

    // --- floor: do not fly fast with the nacelles near vertical.
    //     V <= v_max_hover + (v_max_fwd - v_max_hover)*sin(alpha)
    const float span = cfg_.v_max_forward - cfg_.v_max_hover;
    if (span <= 0.1f || V <= cfg_.v_max_hover) {
        a_min = 0.0f;
    } else {
        a_min = asinf(clampf((V - cfg_.v_max_hover) / span, 0.0f, 1.0f));
    }

    if (a_min > a_max) a_max = a_min;  // corridor pinched: hold, do not invert
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

float FlightKinematics::wingLift(const VehicleState &st) const {
    if (!st.airspeed_valid || st.q_dyn < 1.0f) return 0.0f;
    const float V   = fmaxf(st.airspeed, 1.0f);
    const float fpa = atan2f(st.climb_rate, V);        // flight path angle
    const float aoa = clampf(st.pitch - fpa, -0.35f, 0.35f);
    const float CL  = clampf(cfg_.CL_alpha * aoa, -cfg_.CL_max, cfg_.CL_max);
    return st.q_dyn * cfg_.wing_area * CL;
}

// ===========================================================================
// mode machine  (hysteresis is mandatory - a single distance threshold chatters)
// ===========================================================================
void FlightKinematics::updateMode(const VehicleState &st) {
    if (failsafe_) { mode_ = FlightMode::FAILSAFE; return; }
    if (!armed_)   { mode_ = FlightMode::DISARMED; return; }

    const bool  far   = st.nav_valid && (st.distance_to_go > cfg_.dist_to_forward);
    const bool  near_ = st.nav_valid && (st.distance_to_go < cfg_.dist_to_hover);

    switch (mode_) {
        case FlightMode::DISARMED:
            mode_ = FlightMode::HOVER;
            break;

        case FlightMode::HOVER:
            if (far) mode_ = FlightMode::TRANS_FWD;
            break;

        case FlightMode::TRANS_FWD:
            if (near_)                              mode_ = FlightMode::TRANS_BACK;
            else if (alpha_cmd_ > kHalfPi - 0.03f)  mode_ = FlightMode::FORWARD;
            break;

        case FlightMode::FORWARD:
            if (near_) mode_ = FlightMode::TRANS_BACK;
            break;

        case FlightMode::TRANS_BACK:
            if (far)                     mode_ = FlightMode::TRANS_FWD;
            else if (alpha_cmd_ < 0.03f) mode_ = FlightMode::HOVER;
            break;

        default:
            break;
    }

    switch (mode_) {
        case FlightMode::HOVER:      alpha_target_ = 0.0f;    break;
        case FlightMode::TRANS_FWD:
        case FlightMode::FORWARD:    alpha_target_ = kHalfPi; break;
        case FlightMode::TRANS_BACK: alpha_target_ = 0.0f;    break;
        default:                     alpha_target_ = alpha_cmd_; break;
    }
}

// ===========================================================================
// nacelle schedule: slew toward the target, clamped into the conversion corridor
// ===========================================================================
float FlightKinematics::scheduleAlpha(float dt, const VehicleState &st, ActuatorCmd &out) {
    // Airspeed source, with a ground-speed fallback if the pitot is dead.
    float V      = -1.0f;
    bool  V_good = false;
    if (st.airspeed_valid) {
        V = st.airspeed;  V_good = true;
    } else if (st.nav_valid) {
        V = st.ground_speed;  V_good = true;   // no wind correction: conservative
    }

    float step = cfg_.alpha_rate_nominal * dt;
    float want = alpha_target_;

    // With no speed reference at all we cannot corridor-protect. Freeze the
    // nacelles rather than guess - a frozen tiltrotor still flies, a wrongly
    // converted one does not.
    if (!V_good) {
        out.alpha_min = alpha_cmd_;
        out.alpha_max = alpha_cmd_;
        out.corridor_limited = true;
        return alpha_cmd_;
    }

    float a_new = alpha_cmd_;
    if (want > alpha_cmd_)      a_new = fminf(alpha_cmd_ + step, want);
    else if (want < alpha_cmd_) a_new = fmaxf(alpha_cmd_ - step, want);

    float a_min, a_max;
    corridor(V, st.rho, a_min, a_max);

    const float a_clamped = clampf(a_new, a_min, a_max);
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

    // F_up = sum T_i*cos(alpha_i)
    B[V_FUP][U_THRUST]   =  ca;
    B[V_FUP][U_DA_PITCH] = -T * sa;

    // F_x = sum T_i*sin(alpha_i)
    B[V_FX][U_THRUST]   = sa;
    B[V_FX][U_DA_PITCH] = T * ca;

    // L = l_y*( T_L*cos(a_L) - T_R*cos(a_R) )
    B[V_L][U_DTHRUST] =  cfg_.l_y * ca;
    B[V_L][U_DA_YAW]  = -cfg_.l_y * T * sa;
    B[V_L][U_AILERON] =  qS * b_ * cfg_.Cl_da;

    // M = -l_z*sum T_i*sin(a_i) + l_x*sum T_i*cos(a_i)
    B[V_M][U_THRUST]    = -cfg_.l_z * sa + cfg_.l_x * ca;
    B[V_M][U_DA_PITCH]  = -T * (cfg_.l_z * ca + cfg_.l_x * sa);
    B[V_M][U_ELEVATOR]  =  qS * c_ * cfg_.Cm_de;

    // N = l_y*( T_L*sin(a_L) - T_R*sin(a_R) )
    B[V_N][U_DTHRUST] = cfg_.l_y * sa;
    B[V_N][U_DA_YAW]  = cfg_.l_y * T * ca;
    B[V_N][U_RUDDER]  = qS * b_ * cfg_.Cn_dr;

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
                                const float pri[NV], float u[NU]) {
    const float w[NU] = {
        cfg_.w_thrust, cfg_.w_dthrust, cfg_.w_dalpha_pitch, cfg_.w_dalpha_yaw,
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
    const float lo[NU] = { T_tot_min, -dT_max, -cfg_.nacelle_pitch_band,
                           -cfg_.nacelle_yaw_band, -1.0f, -1.0f, -1.0f };
    const float hi[NU] = { T_tot_max,  dT_max,  cfg_.nacelle_pitch_band,
                            cfg_.nacelle_yaw_band, 1.0f, 1.0f, 1.0f };

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

    if (mode_ == FlightMode::DISARMED || mode_ == FlightMode::FAILSAFE) {
        pid_rate_roll_.reset();  pid_rate_pitch_.reset();  pid_rate_yaw_.reset();
        pid_climb_hover_.reset(); pid_climb_fwd_.reset();  pid_speed_fwd_.reset();
        out = ActuatorCmd{};
        out.mode      = mode_;
        out.alpha_cmd = alpha_cmd_;
        out.nacelle_left  = nac_left_prev_;
        out.nacelle_right = nac_right_prev_;
        return;
    }

    // ---------------- air data ----------------
    const float q_dyn = (st.airspeed_valid && st.q_dyn > 0.0f) ? st.q_dyn : 0.0f;
    const float V     = st.airspeed_valid ? fmaxf(st.airspeed, 0.0f) : 0.0f;

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

    // ---------------- attitude demands ----------------
    // The stabiliser ALWAYS regulates roll and pitch. The config flags only
    // decide whether guidance is allowed to command a non-zero attitude in
    // helicopter mode - "unused DOF" means "not commanded", never "not held".
    const float lim = cfg_.hover_attitude_limit;
    const float roll_hover  = cfg_.hover_allow_roll_cmd
                                ? clampf(sp.roll_sp, -lim, lim) : 0.0f;
    const float pitch_hover = cfg_.hover_allow_pitch_cmd
                                ? clampf(sp.pitch_sp, -lim, lim) : 0.0f;

    const float roll_fwd  = sp.roll_sp;
    const float pitch_fwd = clampf(sp.pitch_sp + pitch_frm_clmb, -0.45f, 0.45f);

    const float roll_cmd  = lerpf(roll_hover,  roll_fwd,  w_fwd);
    const float pitch_cmd = lerpf(pitch_hover, pitch_fwd, w_fwd);

    // Turn demand: direct in helicopter mode, bank-coordinated in airplane mode.
    float psi_dot_fwd = sp.yaw_rate_sp;
    if (V > 5.0f) psi_dot_fwd = (cfg_.g / V) * tanf(clampf(st.roll, -1.0f, 1.0f));
    const float psi_dot = lerpf(sp.yaw_rate_sp, psi_dot_fwd, w_fwd);

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
    const float lift_w = wingLift(st);

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
    allocate(B, v, pri, u);

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

    float nac_L = alpha + u[U_DA_PITCH] + u[U_DA_YAW];
    float nac_R = alpha + u[U_DA_PITCH] - u[U_DA_YAW];
    nac_L = clampf(nac_L, -0.10f, kHalfPi + 0.10f);
    nac_R = clampf(nac_R, -0.10f, kHalfPi + 0.10f);

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
                                       const Setpoints &sp, VehicleState *out_state) {
    static const float kDegToRad = 0.017453292f;

    VehicleState st;

    // --- attitude: Fusion's raw NWU -> this module's internal convention ---
    st.roll  =  raw.fusion_roll_deg  * kDegToRad;   // unchanged
    st.pitch = -raw.fusion_pitch_deg * kDegToRad;   // raw nose-down+ -> nose-up+
    st.yaw   = -raw.fusion_yaw_deg   * kDegToRad;   // raw nose-left+ -> nose-right+
    st.p     =  raw.fusion_gyro_x_dps * kDegToRad;  // unchanged
    st.q     = -raw.fusion_gyro_y_dps * kDegToRad;
    st.r     = -raw.fusion_gyro_z_dps * kDegToRad;

    // --- vertical channel ---
    st.altitude   = raw.baro_altitude;
    st.climb_rate = raw.baro_climb_rate;

    // Air density from the ideal gas law using real pressure/temperature
    // instead of a fixed 1.225 - matters once the pitot is live, since
    // dynamic-pressure-derived airspeed and the expectedDynamicPressure()
    // drag balance both scale with it.
    if (raw.baro_valid && raw.baro_pressure_pa > 1000.0f) {
        const float T_kelvin = raw.baro_temperature_c + 273.15f;
        const float R_specific_air = 287.05f;  // J/(kg*K), dry air
        st.rho = raw.baro_pressure_pa / (R_specific_air * T_kelvin);
    } else {
        st.rho = 1.225f;  // ISA sea-level fallback if baro isn't reporting
    }

    // --- navigation ---
    // Deliberately gated on `nav_ready` (from guidance/F-Code), NOT on raw
    // GPS fix quality - see the RawSensors comment for why.
    st.nav_valid      = raw.nav_ready;
    st.distance_to_go = raw.distance_to_go;
    st.ground_speed   = raw.gnss_ground_speed;

    // --- air data ---
    st.airspeed_valid = raw.airspeed_valid;
    st.airspeed        = raw.airspeed;
    st.q_dyn            = raw.q_dyn;

    ActuatorCmd out;
    update(dt, st, sp, out);

    if (out_state) *out_state = st;
    return out;
}

}  // namespace raven
