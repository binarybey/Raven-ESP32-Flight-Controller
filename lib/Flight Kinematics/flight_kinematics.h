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
// an independent 321-sequence DCM implementation (lib/Attitude
// Transformation) on the identical rotation; both report the identical
// inverted numbers). This assumes the IMU is mounted with its x axis
// pointing forward and z up.
//
// Fusion's yaw is referenced to MAGNETIC north. The mission frame is TRUE
// north, so runCycle() adds VehicleConfig::magnetic_declination_rad.
//
// The IMU module's own axes are x=forward, y=left, z=up, i.e. already NWU -
// mount it with its x arrow toward the nose.
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
// Nacelle i's TILT AXIS crosses the rotor shaft at r_i = (l_x, +/- l_y, -l_z)
// relative to the CG, and the rotor produces
//         F_i = T_i * ( sin(alpha_i), 0, -cos(alpha_i) )
// Only the tilt-axis point matters, not where the prop sits along the shaft:
// the hub is at r_i + h*(sin, 0, -cos), and that extra offset is parallel to
// F_i, so its cross product with F_i is zero. (If the shaft does NOT pass
// through the tilt axis, that offset adds an alpha-dependent moment term.)
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
//   guidance (mission phase, distance-to-go, altitude, airspeed)
//        |
//        v  outer loops, blended by w_fwd = clamp(mu * sin(alpha)),
//        |  mu = clamp(q_dyn / q_ref)
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
// MISSION LIFECYCLE (runCycle(), when an F-Code mission is loaded)
// ============================================================================
//
//   arm() ─► TAKEOFF: hold position over the arming point, nose along the
//   │        first leg, climb to 90% of F43's ground clearance
//   ▼
//   ENROUTE: follow the F-Code path. The mode machine converts to airplane
//   │        mode only with a live pitot and more than dist_to_forward of
//   │        MISSION left, and back to hover inside dist_to_hover of the
//   │        mission end. Along-track speed tapers to zero at the final point.
//   ▼
//   LAND:    hold position over the final point, descend once settled in
//            hover, disarm on touchdown.
//
// FAILSAFE (any phase, latched until disarm): convert back to hover, hold
// position if GNSS is still valid (else wings level), descend, disarm on
// touchdown. Triggered by the "land" command, GNSS lost for longer than
// nav_loss_land_timeout_s, DEM terrain unavailable for longer than
// terrain_loss_land_timeout_s, IMU failure, or barometer failure.
//
// ALTITUDE REFERENCE: the F43 clearance is held above the DEM terrain
// (GLO-30), not above the arming point: AGL = MSL altitude - max terrain
// elevation under the vehicle and along a look-ahead stretch of the path
// (main.cpp TaskTerrain). The MSL altitude is baro for short-term motion,
// continuously re-calibrated by GNSS, so baro errors (non-ISA temperature,
// humidity, weather drift) can't accumulate.
//
// ============================================================================

#pragma once

#include <stdint.h>

#include "pid_f32.h"
#include "fcode_interpreter.h"

namespace raven {

// ---------------------------------------------------------------------------
// Modes
// ---------------------------------------------------------------------------
enum class FlightMode : uint8_t {
    DISARMED   = 0,  // outputs off
    HOVER      = 1,  // alpha parked at 0
    TRANS_FWD  = 2,  // helicopter -> airplane
    FORWARD    = 3,  // alpha parked at 90 deg
    TRANS_BACK = 4,  // airplane -> helicopter
    FAILSAFE   = 5   // controlled landing at the current position - motors stay ON
};

enum class MissionPhase : uint8_t {
    NONE    = 0,  // no mission loaded: manual/RC setpoints
    TAKEOFF = 1,
    ENROUTE = 2,
    LAND    = 3
};

enum class FailsafeReason : uint8_t {
    NONE      = 0,
    COMMANDED = 1,  // operator "land" command
    NAV_LOST  = 2,  // GNSS position stale for longer than nav_loss_land_timeout_s
    IMU       = 3,  // no fresh IMU sample for imu_fail_timeout_s
    BARO      = 4,  // no fresh barometer sample for baro_fail_timeout_s
    TERRAIN   = 5   // DEM elevation unavailable for terrain_loss_land_timeout_s
};

const char *flightModeName(FlightMode m);
const char *missionPhaseName(MissionPhase p);
const char *failsafeReasonName(FailsafeReason r);

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------
struct VehicleState {
    // attitude - AVIATION convention (roll: right-down+, pitch: nose-up+,
    // yaw: nose-right+, compass-style: 0=TRUE North, clockwise-positive).
    // Populate via FlightKinematics::runCycle() from raw sensor floats - see
    // the note above and RawSensors below. Do not populate this struct by
    // hand from raw Fusion output.
    float roll  = 0.0f;   // rad
    float pitch = 0.0f;   // rad
    float yaw   = 0.0f;   // rad, true heading

    // body rates, gyro, already low-passed - same convention/bridge as above
    float p = 0.0f, q = 0.0f, r = 0.0f;   // rad/s

    // air data (nose pitot)
    float q_dyn    = 0.0f;    // Pa, differential pressure, zero-offset removed
    float airspeed = 0.0f;    // m/s, = sqrt(2*q_dyn/rho); 0 if invalid
    float rho      = 1.225f;  // kg/m^3

    // vertical channel
    float altitude   = 0.0f;  // m, MSL-calibrated (baro fused with GNSS)
    float climb_rate = 0.0f;  // m/s, positive up
    float agl        = 0.0f;  // m, clearance above the DEM terrain (highest point
                              //    under the vehicle and along the look-ahead) when
                              //    terrain_valid; otherwise height above the arming
                              //    point (fallback, telemetry only during a mission)
    bool  terrain_valid  = false;
    float terrain_elev_m = 0.0f;  // m MSL, the terrain the clearance is measured to

    // navigation (local mission frame: x east, y north, m)
    float pos_x = 0.0f, pos_y = 0.0f;
    float distance_to_go         = 0.0f;  // m, remaining on the active SEGMENT
    float mission_distance_to_go = 0.0f;  // m, remaining over the WHOLE mission -
                                          //    drives the conversion hysteresis
    float ground_speed       = 0.0f;  // m/s, GNSS speed over ground
    float course_over_ground = 0.0f;  // rad, true, direction of travel (GNSS)

    bool airspeed_valid   = false;
    bool nav_valid        = false;   // position estimate fresh (GNSS within timeout)
    bool ground_vel_valid = false;   // GNSS velocity fresh
};

struct Setpoints {
    float roll_sp       = 0.0f;  // rad, bank demand - direct manual override
                                  // when path_active is false
    float pitch_sp      = 0.0f;  // rad, pitch demand - direct manual override
                                  // when path_active is false
    float yaw_rate_sp   = 0.0f;  // rad/s, EARTH-referenced turn rate - direct
                                  // manual override when path_active is false
    float climb_rate_sp = 0.0f;  // m/s, positive up
    float airspeed_sp   = 0.0f;  // m/s, forward-flight target

    // --- path-following guidance (F-Code / lateral position-hold cascade) ---
    // When path_active is true, these DRIVE roll_sp/pitch_sp/yaw_rate_sp
    // internally through the guidance cascade (see update()) - the three
    // fields above are ignored in that case, not summed with it.
    bool  path_active        = false;
    float cross_track_error  = 0.0f;   // m, signed lateral deviation from the
                                        // path, RIGHT of desired_course positive
    float desired_course     = 0.0f;   // rad, true compass bearing of the path
                                        // tangent (also the hover nose direction)
    float along_speed_sp     = 0.0f;   // m/s, SIGNED along-track ground velocity
                                        // for hover/hybrid flight (negative = back
                                        // up toward a hold point)

    bool  descending         = false;  // landing descent active - arms the
                                        // touchdown detector
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
    bool           armed     = false;
    FlightMode     mode      = FlightMode::DISARMED;
    MissionPhase   phase     = MissionPhase::NONE;
    FailsafeReason fs_reason = FailsafeReason::NONE;
    float alpha_cmd = 0.0f;      // rad, collective nacelle schedule
    float alpha_min = 0.0f;      // rad, corridor floor at current airspeed
    float alpha_max = 0.0f;      // rad, corridor ceiling at current airspeed
    bool  corridor_limited = false;
    bool  air_data_synthetic = false;  // pitot invalid in converted flight:
                                       // q/V estimated from GNSS ground speed
    float mu = 0.0f;             // aero authority fraction, 0..1
    float v_demand[5] = {0};     // [F_up, F_x, L, M, N] before allocation
    float u_applied[7] = {0};    // allocator output after saturation
    float wing_lift_est = 0.0f;  // N
    float thrust_total  = 0.0f;  // N

    // ---- guidance cascade debug (only meaningful if path_active) ----
    float heading_error_dbg   = 0.0f;  // rad, wrapped heading_sp - yaw
    float cross_track_dbg     = 0.0f;  // m
    float lateral_vel_est_dbg = 0.0f;  // m/s, from GNSS course over ground
    float lateral_vel_sp_dbg  = 0.0f;  // m/s, hover cascade outer-loop output
    float along_vel_est_dbg   = 0.0f;  // m/s
    float along_vel_sp_dbg    = 0.0f;  // m/s
};

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
struct VehicleConfig {
    // ---- mass / inertia (about the CENTER OF MASS, BODY axes) ----
    // DESIGN ESTIMATE at MTOW 2.5 kg (the stated absolute maximum - using it
    // keeps the conversion corridor conservative). Built from:
    //   airframe, CAD "V-22 Osprey v24": 0.567 kg, CAD inertia (below)
    //   battery 4S5P 18650 + wiring ~1.0 kg near the CG (box ~72x90x65 mm)
    //   2 nacelles x ~0.28 kg at +/-l_y (motor, 3-blade prop, tilt servo +
    //     mechanism, ESC)
    //   avionics ~0.15 kg, surface servos ~0.05 kg, rest = margin at the CG
    // Replace with a CAD export once the components are modelled (plain
    // blocks with the right mass and position are enough). Every gain is
    // inertia-normalised, so these four numbers are all the controller needs.
    //
    // CAD -> BODY AXES. The CAD frame has +Y = nose, +Z = up, +X = right
    // wing (the model "faces west" in NWU terms). The CAD file itself proves
    // it: CoM X = -0.003 mm and Ixy, Ixz ~ 0 (mirror-symmetric in X, so X is
    // the span axis); Iyz = -13765 g*mm^2 is the one product a symmetric
    // aircraft has (fore-aft x vertical). So the inertia about the NOSE axis
    // (roll, body Ixx) is CAD Iyy, and about the SPAN axis (pitch, body Iyy)
    // is CAD Ixx. Same numbers, different axis names - roll IS the Ixx field.
    //   airframe only:  roll 4.342e-3, pitch 3.009e-3, yaw 6.770e-3 kg*m^2
    //   nacelles add 2*0.28*0.18^2 = 0.018 to roll and yaw.
    float mass = 2.5f;         // kg
    float Ixx  = 0.024f;       // kg*m^2, roll  (airframe 0.0043 + nacelles 0.018 + battery/servos)
    float Iyy  = 0.0065f;      // kg*m^2, pitch (airframe 0.0030 + battery 0.0010 + nacelles 0.0014 + tail servos)
    float Izz  = 0.027f;       // kg*m^2, yaw   (airframe 0.0068 + nacelles 0.018 + battery/servos)

    // ---- nacelle geometry relative to the CG (see the derivation at the top) ----
    // Measured to the nacelle TILT AXIS where the prop shaft crosses it - not
    // to the propeller. In the CAD frame: l_x = Y_axis - Y_cg,
    // l_y = |X_axis - X_cg|, l_z = Z_axis - Z_cg, using the FINAL CG (battery
    // etc. installed), not the airframe-only CoM.
    float l_y = 0.180f;     // m, lateral distance to each nacelle's shaft line.
                            //    Sets hover ROLL authority (differential thrust
                            //    * l_y) and hover YAW authority (differential tilt).
                            //    ~ half span 168 mm + nacelle radius; confirm in CAD.
    float l_z = 0.05f;      // m, tilt axis ABOVE the CG. DESIGN TARGET 40-60 mm.
                            //    Hover PITCH authority comes only from tilting both
                            //    nacelles together (no cyclic on fixed props):
                            //    M = T_total * l_z * sin(nacelle_pitch_band). At
                            //    2.5 kg, 50 mm and +/-10 deg that's 0.21 N*m ->
                            //    ~33 rad/s^2 with Iyy above. l_z -> 0 means NO hover
                            //    pitch control. A lower battery raises l_z.
    float l_x = 0.0f;       // m, tilt axis AHEAD of the CG. DESIGN TARGET 0: the
                            //    hover thrust line must pass through the CG, or a
                            //    steady tilt of atan(l_x/l_z) is spent just trimming
                            //    (10 mm with l_z 50 mm = 11 deg - the whole pitch
                            //    band). Keep |l_x| <= ~0.2*l_z; move the battery to get it.

    // ---- wing / aero ----
    // Rectangular NACA 23018 wing (CAD). Coefficients are estimates until the
    // Fluent results: whole-aircraft CL vs body angle of attack gives CL_0,
    // CL_alpha, CL_max; the drag polar gives CD0 and oswald_e.
    float wing_span    = 0.33625f; // m
    float mean_chord   = 0.087f;   // m
    float wing_area    = 0.029254f;// m^2, span * chord
    float aspect_ratio = 3.865f;   // span / chord
    float oswald_e     = 0.80f;
    float CL_0         = 0.08f;    // CL at zero BODY angle of attack: 23018 camber
                                   // (zero-lift angle ~ -1.2 deg), zero wing incidence
                                   // assumed
    float CL_max       = 1.10f;    // finite wing, Re ~1.5e5 - low-Re, check in Fluent
    float CL_alpha     = 3.82f;    // per rad, Helmbold estimate for AR 3.87
    float CD0          = 0.045f;

    // Control-surface effectiveness, expressed so that a POSITIVE command
    // produces a POSITIVE moment. Handle servo reversal in your PWM layer,
    // not here. Units: moment coefficient per unit command (-1..1).
    float Cl_da = 0.18f;
    float Cm_de = 0.55f;
    float Cn_dr = 0.08f;

    // ---- propulsion ----
    // ESTIMATE until a static thrust test: momentum theory (figure of merit
    // 0.55, 80% motor+ESC) for an 11-inch 3-blade prop at 1500 m, limited by a
    // 4S5P pack at 14 A/cell (70 A). thrust/weight 1.47 at 2.5 kg - 4P would
    // only give ~1.26, below what validateConfig() accepts.
    float thrust_max_per_rotor = 18.0f;   // N, static, at full throttle
    float thrust_min_per_rotor = 0.5f;    // N, keep ESCs spinning when armed
    float nacelle_rate_max     = 0.70f;   // rad/s, servo slew limit
    float nacelle_pitch_band   = 0.17f;   // rad (~10 deg) max collective
                                          // perturbation available to pitch
    float nacelle_yaw_band     = 0.14f;   // rad (~8 deg) max differential tilt

    // ---- conversion corridor ----
    float stall_margin  = 1.25f;   // V >= margin * V_stall(alpha)
    float v_max_hover   = 8.0f;    // m/s, airspeed ceiling with nacelles vertical
    float v_max_forward = 34.0f;   // m/s, airframe Vne - PLACEHOLDER
    // Fraction of max total thrust the corridor CEILING may assume goes to
    // lift. At 1.0 the corridor allows tilting until the rotors are at full
    // throttle just holding altitude - nothing left for attitude control.
    float corridor_thrust_fraction = 0.75f;

    // ---- transition scheduling (HYSTERESIS - a single threshold chatters) ----
    // Compared against the distance left in the WHOLE mission, not the
    // current segment, so the aircraft doesn't reconvert at every segment end.
    float dist_to_forward   = 400.0f;  // m, beyond this go airplane
    float dist_to_hover     = 250.0f;  // m, inside this come back
    float alpha_rate_nominal = 0.25f;  // rad/s, commanded conversion rate

    // ---- allocation ----
    float q_ref      = 480.0f;   // Pa, dynamic pressure at which control
                                 // surfaces are considered fully effective.
                                 // ~= 0.5*rho*V_cruise^2 (0.5*1.225*28^2).
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

    // ---- lateral position-hold cascade (path_active guidance) ----
    float lateral_hold_max_tilt   = 0.20f;  // rad (~11.5deg), max hover roll/pitch
                                            // tilt commanded by the position-hold cascade
    float max_bank_fwd            = 0.52f;  // rad (~30deg), max forward-flight bank
                                            // angle commanded by path-following guidance
    float max_course_correction   = 0.79f;  // rad (~45deg), max aim-off angle the
                                            // forward-mode cross-track loop may add
                                            // to desired_course to converge on the path

    // ---- F-Code mission guidance ----
    float cruise_ground_speed_hover = 5.0f;  // m/s, hover/hybrid along-track target
    float cruise_airspeed           = 28.0f; // m/s, forward-flight target -
                                             // PLACEHOLDER, keep >= stall_margin *
                                             // V_stall (validateConfig() checks)
    float approach_decel            = 1.0f;  // m/s^2, along-track speed tapers as
                                             // sqrt(2*a*d) into the final point
    float point_hold_gain           = 0.4f;  // 1/s, along-track position error ->
                                             // velocity, for takeoff/landing holds
    float point_hold_max_speed      = 2.0f;  // m/s
    float takeoff_complete_fraction = 0.9f;  // TAKEOFF ends at this fraction of the
                                             // F43 ground clearance
    float land_descent_rate         = 0.8f;  // m/s, landing / failsafe descent.
                                             // Constant, because AGL is relative to
                                             // the ARMING point and the terrain
                                             // under the landing point may differ.
    float land_settle_radius_m      = 3.0f;  // m, start descending once within this
                                             // of the final point and slow
    float land_settle_timeout_s     = 30.0f; // s, descend anyway after this long
    float landed_confirm_s          = 2.0f;  // s of (still + low thrust) = touchdown
    float land_detect_delay_s       = 5.0f;  // s of commanded descent before touchdown
                                             // detection arms. At the START of a descent
                                             // thrust drops before the 2 Hz baro climb-rate
                                             // estimate shows any motion - without this
                                             // delay that looks exactly like touchdown and
                                             // would disarm in mid-air.

    // Magnetic declination, EAST positive. Fusion's yaw is magnetic; the
    // mission frame is true north. NOAA WMMHR near Ankara, 2026-09:
    // 6deg 8' 37" E. It varies ~5-7 deg E across Turkey - use the value for
    // the flying area, and re-check yearly.
    float magnetic_declination_rad = 0.107227f;   // +6.1436 deg

    // GPS position handling: dead-reckoning between fixes using the GNSS
    // velocity VECTOR (speed + course over ground), then corrected toward
    // each GENUINELY NEW fix (RawSensors::gnss_fix_seq). tau sets how fast
    // that correction pulls the estimate toward a fresh fix - short enough
    // to track real drift, long enough not to re-inject raw GPS noise.
    float gnss_position_correction_tau_s = 1.5f;   // s

    // If no NEW fix has arrived in this long, stop trusting the dead-
    // reckoned estimate - nav_valid goes false (mode machine returns to
    // hover, attitude goes level).
    float gnss_fix_stale_timeout_s = 4.0f;   // s
    // ...and if it stays lost this long after that, land (FAILSAFE NAV_LOST).
    float nav_loss_land_timeout_s  = 20.0f;  // s

    // Barometer/GNSS altitude complementary filter: baro's climb_rate drives
    // the fast/responsive part, GNSS altitude pulls out the baro's errors.
    // Those grow with altitude CHANGE, not just time: pressure altitude
    // assumes the ISA temperature profile, so a 1000 m climb on a day 20 C
    // warmer than ISA reads ~7% short; humidity (virtual temperature) adds
    // under 1%, weather drift ~8 m per hPa. The in-flight tau is short enough
    // to keep that below a few metres at 3 m/s climb, long enough to average
    // GNSS vertical noise. On the ground (disarmed) a faster tau calibrates
    // the offset before takeoff - pre-arm waits for it (prearm_gnss_settle_s).
    float baro_gnss_fusion_tau_s        = 20.0f;   // s, armed
    float baro_gnss_fusion_tau_ground_s = 10.0f;   // s, disarmed

    // Second, FASTER correction: bounds how long a small bias in the
    // climb_rate estimate itself can accumulate. It pulls toward
    // baro_altitude MINUS the separately-tracked bias estimate (baro_bias_m_),
    // so its target is itself already GNSS-calibrated.
    float baro_resync_tau_s = 4.0f;   // s

    // ---- sensor health ----
    float imu_fail_timeout_s  = 0.1f;   // s without a fresh IMU sample -> FAILSAFE
    float baro_fail_timeout_s = 2.0f;   // s without a fresh baro sample -> FAILSAFE
    // DEM lookups failing while the position is still known (SD fault, off the
    // tiles): hold the current MSL altitude, then land if it doesn't recover.
    float terrain_loss_land_timeout_s = 5.0f;

    // ---- pre-arm ----
    float   prearm_max_tilt_rad = 0.26f;  // ~15 deg
    uint8_t prearm_min_sats     = 6;      // mission only
    float   prearm_max_hdop     = 2.5f;   // mission only
    float   prearm_max_start_distance_m = 50.0f;  // mission only: must arm near F90
    float   prearm_gnss_settle_s = 30.0f;  // mission only: GNSS + baro together this long
                                           // (3 ground taus) so the MSL altitude the DEM
                                           // clearance depends on has converged

    // ---- misc ----
    float g = 9.80665f;
};

// Result bits of FlightKinematics::validateConfig().
enum ConfigIssue : uint32_t {
    CFG_BAD_MASS_INERTIA   = 1u << 0,  // fatal: non-positive mass or inertia
    CFG_HOVER_THRUST       = 1u << 1,  // fatal: hover needs > 75% of max thrust
    CFG_NO_FULL_CONVERSION = 1u << 2,  // warning: the wing can't carry the weight at Vne -
                                       //   forward flight stays partially converted
    CFG_CRUISE_BELOW_STALL = 1u << 3,  // warning: cruise_airspeed < stall_margin * wing-only
                                       //   stall speed - rotors carry part of the weight
    CFG_FATAL_MASK         = CFG_BAD_MASS_INERTIA | CFG_HOVER_THRUST
};
const char *configIssueText(uint32_t single_bit);

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

    // --- path-following cascade (path_active guidance only) ---

    // heading-hold: wrapped heading error (rad) -> yaw rate demand (rad/s).
    PidGains heading_hold{ 1.2f, 0.0f, 0.0f, 1.0f, 10.0f, -0.6f, 0.6f };

    // HOVER cascade, stage 1 (outer): cross-track error (m) -> lateral
    // velocity setpoint (m/s). Deliberately no D-term - the inner stage
    // below provides the damping a cascade needs.
    PidGains xtrack_to_latvel_hover{ 0.35f, 0.02f, 0.0f, 2.0f, 0.0f, -3.0f, 3.0f };

    // HOVER cascade, stage 2 (inner): lateral velocity error (m/s) -> roll
    // tilt (rad). Lateral velocity comes from the GNSS velocity vector
    // (course over ground), NOT from heading - heading hold keeps the nose on
    // the course, so a heading-based estimate would always read ~0. Output
    // clamp is set from lateral_hold_max_tilt at runtime.
    PidGains latvel_to_roll_hover{ 0.12f, 0.02f, 0.01f, 0.1f, 12.0f, -1.0f, 1.0f };

    // HOVER along-track velocity hold: signed along-track velocity error
    // (m/s) -> pitch tilt (rad). Parallels speed_fwd for low-speed flight.
    PidGains speed_hover{ 0.10f, 0.02f, 0.0f, 0.1f, 8.0f, -1.0f, 1.0f };

    // FORWARD mode: cross-track error (m) -> course-correction angle (rad),
    // single-stage (forward flight's own coordinated-turn dynamics already
    // provide cascade-equivalent damping). Output clamp set from
    // max_course_correction at runtime.
    PidGains xtrack_to_course_fwd{ 0.010f, 0.0008f, 0.0f, 0.3f, 3.0f, -1.0f, 1.0f };

    // Altitude-hold: AGL error (m) -> climb rate demand (m/s).
    PidGains altitude_hold{ 0.40f, 0.05f, 0.05f, 3.0f, 5.0f, -3.0f, 3.0f };
};

// ---------------------------------------------------------------------------
// RateEstimator - low-pass derivative of a slowly-sampled signal (e.g.
// barometer altitude at ~2Hz). Call update() only when a genuinely NEW
// sample has arrived - calling it every control-loop tick with a repeated
// stale value will bias the estimate toward zero between samples. Owned and
// driven by whichever task reads that sensor (see main.cpp's TaskBMP).
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
// Every group carries its own validity flag - never fill a group with
// defaults and mark it valid.
// ---------------------------------------------------------------------------
struct RawSensors {
    // --- IMU / AHRS ---
    bool  imu_valid  = false;   // a FRESH IMU sample was fused this cycle
    bool  mag_valid  = false;   // magnetometer fresh and used by the AHRS
    bool  ahrs_ready = false;   // Fusion has finished its startup convergence
    // Fusion's RAW output, uncorrected. Pass euler.angle.roll/pitch/yaw and
    // the BIAS-CORRECTED gyroscope vector (the one actually fed to
    // FusionAhrsUpdate) directly - do not pre-negate anything yourself.
    float fusion_roll_deg  = 0.0f;
    float fusion_pitch_deg = 0.0f;
    float fusion_yaw_deg   = 0.0f;   // MAGNETIC - runCycle() applies declination
    float fusion_gyro_x_dps = 0.0f;
    float fusion_gyro_y_dps = 0.0f;
    float fusion_gyro_z_dps = 0.0f;

    // --- barometer --- (baro_valid = a sample newer than baro_fail_timeout_s)
    bool  baro_valid      = false;
    float baro_altitude   = 0.0f;   // m
    float baro_climb_rate = 0.0f;   // m/s, positive up - compute via RateEstimator
                                     // at the barometer's own sample rate, not here
    float baro_pressure_pa    = 101325.0f;  // Pa, for air-density calc
    float baro_temperature_c  = 15.0f;      // deg C, for air-density calc

    // --- GNSS position (GGA) ---
    bool     gnss_fix_valid  = false;   // fix quality > 0
    double   gnss_lat_deg    = 0.0;     // signed decimal degrees (N/E positive)
    double   gnss_lon_deg    = 0.0;
    float    gnss_altitude_m = 0.0f;    // m, MSL
    float    gnss_hdop       = 99.9f;
    uint8_t  gnss_sats       = 0;
    // Bumped once per accepted GGA sentence (~1 Hz), NOT every control
    // cycle - lets runCycle() tell "new fix" from "same fix still sitting in
    // the snapshot", which is what makes dead-reckoning between fixes work.
    uint32_t gnss_fix_seq    = 0;

    // --- GNSS velocity (RMC) ---
    bool     gnss_vel_valid     = false;  // RMC status 'A' and course present
    float    gnss_ground_speed  = 0.0f;   // m/s
    float    gnss_course_rad    = 0.0f;   // rad, TRUE course over ground, compass
    uint32_t gnss_vel_seq       = 0;      // bumped once per accepted RMC sentence

    // --- pitot (see hardware_interface.h / main.cpp TaskAirData) ---
    bool  pitot_valid = false;   // fresh, zeroed, plausible
    float pitot_q_pa  = 0.0f;    // Pa, zero-offset removed, filtered

    // --- terrain (DEM, main.cpp TaskTerrain, ~1 Hz) ---
    bool  terrain_valid  = false;  // fresh lookup for the current position
    float terrain_elev_m = 0.0f;   // m MSL: max over the footprint under the vehicle
                                   // AND along the look-ahead stretch of the path
};

// ---------------------------------------------------------------------------
// Main class
// ---------------------------------------------------------------------------
class FlightKinematics {
  public:
    // Returns validateConfig()'s issue bits (0 = clean). A fatal issue
    // (CFG_FATAL_MASK) makes arm() refuse.
    uint32_t begin(const VehicleConfig &cfg, const ControlGains &gains);

    // Call from the core-1 control task, once per IMU sample (100 Hz).
    // `dt` is sanitised internally but feed it from esp_timer_get_time().
    void update(float dt, const VehicleState &st, const Setpoints &sp, ActuatorCmd &out);

    // Single entry point for real hardware use: takes plain sensor floats
    // (see RawSensors above), applies the NWU->internal-convention and
    // declination corrections, fuses altitude, dead-reckons position, runs
    // the mission lifecycle / failsafe logic, and calls update(). When no
    // mission is loaded, falls back to `sp` as given (manual/RC path). Also
    // exposes the corrected VehicleState via `out_state` (nullptr if unused).
    ActuatorCmd runCycle(float dt, const RawSensors &raw, const Setpoints &sp,
                         VehicleState *out_state = nullptr);

    // Arming is refused (returns false, *reason says why) unless the pre-arm
    // checks pass against the most recent runCycle() inputs.
    bool arm(const char **reason);
    void disarm();
    // Latched controlled landing (motors stay on). Ignored while disarmed.
    void requestFailsafe(FailsafeReason why);
    // Result of the boot-time check that DEM tiles cover the whole mission
    // route. arm() refuses a mission without it.
    void setMissionTerrainOk(bool ok) { mission_terrain_ok_ = ok; }
    void reset();   // zero all integrators and latched control state

    bool           armed() const { return armed_; }
    FlightMode     mode() const { return mode_; }
    MissionPhase   phase() const { return phase_; }
    FailsafeReason failsafeReason() const { return fs_reason_; }
    float          alpha() const { return alpha_cmd_; }
    uint32_t       configIssues() const { return config_issues_; }

    // Checks the config for physically impossible or inconsistent values.
    uint32_t validateConfig() const;

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

    // Wing-borne stall speed at full conversion (all weight on the wing).
    float stallSpeed(float rho) const;

  private:
    void  updateMode(const VehicleState &st);
    float scheduleAlpha(float dt, const VehicleState &st, ActuatorCmd &out);
    void  buildEffectiveness(float alpha, float thrust_total, float q_dyn, float B[5][7]) const;
    void  allocate(const float B[5][7], const float v[5], const float pri[5], float u[7]);
    float wingLift(const VehicleState &st, float V, float q_dyn) const;
    float corridorCeiling(float V, float rho) const;

    void      updateNavEstimate(float dt, const RawSensors &raw, VehicleState &st);
    void      updateHealth(float dt, const RawSensors &raw);
    Setpoints runMissionGuidance(float dt, VehicleState &st);   // fills st's distances
    Setpoints runFailsafeGuidance(float dt, const VehicleState &st);
    void      pointHold(const VehicleState &st, float hold_x, float hold_y, float course,
                        Setpoints &sp) const;
    float     clearanceHold(float target_agl, const VehicleState &st, float dt);
    void      resetControllers();

    VehicleConfig cfg_;
    ControlGains  gains_;
    uint32_t      config_issues_ = 0;

    Pid pid_rate_roll_, pid_rate_pitch_, pid_rate_yaw_;
    Pid pid_climb_hover_, pid_climb_fwd_, pid_speed_fwd_;
    Pid pid_heading_hold_;
    Pid pid_xtrack_to_latvel_hover_, pid_latvel_to_roll_hover_, pid_speed_hover_;
    Pid pid_xtrack_to_course_fwd_;
    Pid pid_altitude_hold_;

    // ---- position / velocity estimate (runs armed or not) ----
    bool     pos_est_primed_     = false;
    float    pos_est_x_m_        = 0.0f;
    float    pos_est_y_m_        = 0.0f;
    uint32_t last_gnss_seq_seen_ = 0;
    float    time_since_fix_s_   = 1.0e6f;
    uint32_t last_vel_seq_seen_  = 0;
    float    time_since_vel_s_   = 1.0e6f;
    float    gnss_speed_         = 0.0f;
    float    gnss_course_        = 0.0f;
    bool     gnss_vel_have_      = false;

    // ---- vertical channel (runs armed or not) ----
    bool  alt_fusion_primed_  = false;
    float fused_altitude_m_   = 0.0f;   // MSL-calibrated baro/GNSS estimate
    float baro_bias_m_        = 0.0f;   // estimated (raw.baro_altitude - GNSS altitude)
    // AGL reference, captured at arm() in the RAW-BARO frame (fused + bias),
    // so the bias estimate settling later can't shift the vehicle's idea of
    // its own height.
    bool  ground_ref_set_     = false;
    float ground_ref_baro_m_  = 0.0f;
    float baro_frame_alt_m_   = 0.0f;   // smoothed RAW baro altitude (no GNSS
                                        // correction) for the no-DEM fallback AGL
    RateEstimator gnss_climb_est_{0.2f};   // vertical fallback if the baro dies
    float gnss_alt_settle_s_  = 0.0f;      // continuous GNSS+baro time (pre-arm)
    float time_s_             = 0.0f;      // accumulated runCycle time

    // ---- mission lifecycle ----
    MissionPhase phase_           = MissionPhase::NONE;
    bool   takeoff_hold_set_      = false;
    float  hold_x_ = 0.0f, hold_y_ = 0.0f, hold_course_ = 0.0f;
    float  nav_lost_s_            = 0.0f;
    bool   land_descending_       = false;
    float  land_timer_s_          = 0.0f;
    float  last_target_agl_       = 0.0f;
    bool   mission_terrain_ok_    = false;
    bool   alt_ref_msl_           = false;   // clearance hold fell back to MSL hold
    float  msl_hold_m_            = 0.0f;
    float  terrain_lost_s_        = 0.0f;

    // ---- failsafe / health ----
    FailsafeReason fs_reason_     = FailsafeReason::NONE;
    bool   fs_hold_set_           = false;
    float  imu_bad_s_             = 0.0f;
    float  baro_bad_s_            = 0.0f;
    float  landed_timer_s_        = 0.0f;
    float  descend_time_s_        = 0.0f;

    // ---- last inputs, for arm()'s pre-arm checks ----
    RawSensors   last_raw_;
    VehicleState last_st_;
    bool         have_last_ = false;

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
