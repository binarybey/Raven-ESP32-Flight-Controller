// fcode_interpreter.h - parses an F-Code mission file and, every cycle,
// reports where the vehicle should be going right now.
//
// ============================================================================
// SCOPE / DESIGN
// ============================================================================
// This file knows NOTHING about flight_kinematics.h - no Setpoints, no
// VehicleState, no raven:: namespace. It solves one problem: given the parsed
// mission and the vehicle's current position, produce distance-to-go,
// cross-track error, and desired course for whichever segment is active.
// FlightKinematics::runMissionGuidance() (flight_kinematics.cpp) is the only
// place that turns NavOutput into Setpoints.
//
// COORDINATE FRAME: local flat-tangent-plane meters, origin at the F90
// absolute start position, x=east (longitude-derived), y=north
// (latitude-derived) - matching the F-Code's own X/Y convention directly.
// Proper geodesy: 1 arcsec latitude = 30.87 m; 1 arcsec longitude =
// 30.87*cos(lat) m. Use gpsToLocal() below to convert live GPS fixes into
// this SAME frame - do not convert them any other way, or cross_track_error
// will be measuring against two different coordinate systems.
//
// HEMISPHERE: packed-DMS fields are unsigned, so missions are assumed to be
// in the north-east hemisphere (N latitude, E longitude). A '-' or any other
// non-digit in a DMS field is rejected by begin() rather than misread.
//
// GRAMMAR (confirmed against real mission files):
//   F43; Z##;                          mission start, Z = ground clearance, m
//   F90; X<DDDMMSS>; Y<DDMMSS>;        absolute start, X=lon, Y=lat, packed DMS
//   F01; X<+-ss.s>; Y<+-ss.s>;         line, RELATIVE delta in arcseconds,
//                                       X=Dlongitude, Y=Dlatitude (same axis
//                                       assignment as F90, NOT swapped)
//   F02; I<DMS>; J<DMS>; R<m>; A<deg>; X<DMS>; Y<DMS>;   CW arc
//   F03; ...                           CCW arc, same fields as F02
//   F39;                                mission end
// I/J = arc center, X/Y = arc endpoint (same packed-DMS, X=lon,Y=lat as F90).
// R and A are checked against the geometry computed from I/J/X/Y as a
// sanity warning ONLY (see MissionInfo::arc_warnings) - the live radius/sweep
// this module tracks is always derived from the geometry, never taken from
// R/A verbatim (R/A can be stale or off by whatever the path-generation
// tool's own convergence-correction constant turns out to be).
// An arc whose endpoint equals its start point is a full circle (G-code
// convention).
//
// SEGMENT COMPLETION: by along-track PROGRESS, not proximity. A LINE is done
// once the vehicle crosses the perpendicular through its endpoint (however
// far off to the side it is); an ARC is done once the swept angle reaches
// the arc's sweep. A proximity radius can be missed entirely when the
// vehicle is a few meters off the path, and then the segment never ends.
//
// ============================================================================

#pragma once

#include <stdint.h>

namespace fcode {

enum class Course : unsigned char { LINE, ARC_CW, ARC_CCW };

struct NavOutput {
    // --- state machine ---
    bool en_route          = false;  // true while a segment is actively being tracked
    bool segment_completed = false;  // true for the one cycle a segment finishes on
    bool mission_complete  = false;  // true once the last segment is finished

    Course course        = Course::LINE;
    int    segment_index = 0;        // active segment (the last one once complete)

    // --- position-dependent, recomputed every update() call ---
    float distance_to_go         = 0.0f;  // m, along-track remaining on the CURRENT
                                          // segment (briefly <= 0 on the cycle it completes)
    float mission_distance_to_go = 0.0f;  // m, along-track remaining over the WHOLE
                                          // mission (current + all later segments).
                                          // After completion: SIGNED along-track
                                          // distance to the final point (negative
                                          // once past it) - the hold/landing target.
    float cross_track_error  = 0.0f;      // m, signed, RIGHT of desired_course is positive
    float desired_course_rad = 0.0f;      // rad, compass bearing (0=North, clockwise+)

    // --- set once when the mission is parsed, held steady ---
    float ground_clearance_m = 0.0f;      // m, target height AGL, from F43's Z
};

// Parse summary, filled by begin(). Print it at boot.
struct MissionInfo {
    bool        ok             = false;
    const char *error          = nullptr;  // why begin() failed, nullptr if ok
    int         error_line     = 0;        // 1-based line number of the error (0 = n/a)
    int         segment_count  = 0;
    float       total_length_m = 0.0f;
    float       ground_clearance_m = 0.0f;
    double      origin_lat_deg = 0.0;
    double      origin_lon_deg = 0.0;
    int         unknown_lines  = 0;        // lines with an unrecognised opcode (ignored)
    int         arc_warnings   = 0;        // arcs whose R/A disagree with I/J/X/Y geometry,
                                           // or whose start point is off the circle
    float       worst_arc_radius_err_pct = 0.0f;
    float       worst_arc_sweep_err_deg  = 0.0f;
};

// Call once at boot, after the F-Code .fcode file has been loaded into `text`
// (null-terminated, see loadFCodeFromSd()). Parses the whole mission up
// front into a fixed-size internal segment array and resets to the
// beginning. Returns false (and fills info().error) if the text is not a
// complete, well-formed mission: F43 and F90 must precede the first
// segment, F39 must be present, numeric fields must parse, and the segment
// buffer must not overflow.
bool begin(const char *text);

const MissionInfo &info();

// True when begin() succeeded - a mission is available to fly.
bool loaded();

// True once the last segment has been completed (stays true until begin()).
bool complete();

// Local-frame origin. begin() sets it from F90. With no mission loaded,
// setOrigin() lets the caller anchor the frame at the first GPS fix so local
// positions stay small (float precision) for failsafe position hold.
bool hasOrigin();
void setOrigin(double lat_deg, double lon_deg);

// Converts a live GPS fix (decimal degrees, signed: N/E positive) into the
// SAME local meters frame the mission was parsed into.
void gpsToLocal(double lat_deg, double lon_deg, float &x_m, float &y_m);

// Inverse of gpsToLocal().
void localToGps(float x_m, float y_m, double &lat_deg, double &lon_deg);

// Point on the planned path at along-path distance s_m from the mission
// start (clamped to [0, total length]). Reads only the parsed segments,
// which never change after begin() - safe to call from another task (the
// terrain look-ahead does). Returns false if no mission is loaded.
bool pathPointAt(float s_m, float &x_m, float &y_m);

// Call every control cycle with the vehicle's current LOCAL position (from
// gpsToLocal() above). Advances to the next path-defining line (F01/F02/F03)
// internally once the current one completes - F43/F90/F39 are consumed
// during begin() and never appear here as an active segment. Once the
// mission is complete it keeps reporting hold geometry about the final
// point (see NavOutput::mission_distance_to_go).
void update(float pos_x_m, float pos_y_m, NavOutput &out);

// Reads the named file from the SD card (standard Arduino `SD` library over
// SPI) into a static internal buffer and returns a pointer to it, or nullptr
// on failure (card not present, file not found, file too large for the
// buffer - a truncated mission is never returned). loadError() says which.
// Call once in setup(), before fcode::begin(). The returned buffer is owned
// by this module - do not free it, and it is overwritten by the next call.
const char *loadFCodeFromSd(const char *path, uint8_t csPin);
const char *loadError();

// loadFCodeFromSd() + begin(), printing the mission summary (or why there is
// no mission) on Serial. Returns true if a mission is ready to fly. Either
// step failing leaves no mission loaded - never a half-loaded or truncated one.
bool loadMission(const char *path, uint8_t csPin);

}  // namespace fcode
