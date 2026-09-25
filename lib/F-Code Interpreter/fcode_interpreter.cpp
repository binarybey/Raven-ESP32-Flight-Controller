// fcode_interpreter.cpp - see fcode_interpreter.h for the interface contract
// and grammar. Implementation notes and derivations live here, next to the
// code they justify.

#include "fcode_interpreter.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>
#ifdef FCODE_DEBUG
#include <stdio.h>
#endif

namespace fcode {

namespace {

constexpr int   kMaxSegments = 64;   // generous for any realistic mission file
constexpr float kPi          = 3.14159265f;
constexpr float kTwoPi       = 6.28318531f;
constexpr float kMPerArcsecLat = 30.87f;   // best-verified value so far, good to
                                            // within ~2.9% - still pending the exact
                                            // constant the path-generation tool uses.
constexpr float kCompletionM = 1.0f;       // segment done when <= this much along-track
                                            // distance remains (see header: progress,
                                            // not proximity)

// Arc R/A sanity-check tolerances (warnings only - see header).
constexpr float kArcRadiusTolPct = 5.0f;
constexpr float kArcSweepTolDeg  = 3.0f;

struct Segment {
    Course course;
    float start_x, start_y;      // m, local frame
    float target_x, target_y;    // m, local frame
    float center_x, center_y;    // m, ARC only
    float radius_m;              // ARC only - hypot(start-center), NOT the file's R
    float theta_start;           // ARC only - compass bearing center->start
    float sweep_rad;             // ARC only - (0, 2pi], in the direction of travel
    float length_m;              // along-track length
    float end_course_rad;        // path tangent (compass) at the endpoint
};

Segment segs_[kMaxSegments];
float   remainingAfter_[kMaxSegments];   // sum of lengths of segments i+1..n-1
float   cumStart_[kMaxSegments];         // along-path distance at the start of segment i
int     segCount_ = 0;
int     activeIdx_ = -1;
bool    missionDone_ = false;
bool    loaded_ = false;
float   groundClearanceM_ = 0.0f;
MissionInfo info_;

bool    haveOrigin_ = false;
double  originLatDeg_ = 0.0, originLonDeg_ = 0.0;   // for gpsToLocal()
float   refLatForLonScale_ = 0.0f;                   // deg, for cos(lat) scale

// Arc progress is tracked statefully (accumulated swept angle) so a full
// circle, or a vehicle that briefly drifts behind the arc start, can't be
// mistaken for "already at the end".
bool    arcProgressValid_ = false;
float   arcProgress_  = 0.0f;   // rad swept so far, in the direction of travel
float   arcPhasePrev_ = 0.0f;

// ---------------------------------------------------------------------------
// small helpers - no heap, no STL streams, fixed buffers only
// ---------------------------------------------------------------------------
float wrapPi(float a) {
    while (a >  kPi) a -= kTwoPi;
    while (a < -kPi) a += kTwoPi;
    return a;
}
float wrap2Pi(float a) {
    a = fmodf(a, kTwoPi);
    if (a < 0.0f) a += kTwoPi;
    return a;
}

float mPerArcsecLon(float lat_deg) { return kMPerArcsecLat * cosf(lat_deg * kPi / 180.0f); }

// Packed DMS integer string ("333633" -> 33deg 36min 33sec) to arcseconds.
// Digits only (1-3 degree digits + MMSS); anything else is rejected, which
// also rejects a '-' sign - see the hemisphere note in the header.
bool parseDms(const char *s, long &arcsec) {
    const int len = (int)strlen(s);
    if (len < 5 || len > 7) return false;
    for (int i = 0; i < len; ++i)
        if (s[i] < '0' || s[i] > '9') return false;
    const int sec    = (s[len-2]-'0')*10 + (s[len-1]-'0');
    const int minute = (s[len-4]-'0')*10 + (s[len-3]-'0');
    if (sec >= 60 || minute >= 60) return false;
    long deg = 0;
    for (int i = 0; i < len-4; ++i) deg = deg*10 + (s[i]-'0');
    arcsec = deg*3600L + minute*60L + sec;
    return true;
}

bool parseFloat(const char *s, float &v) {
    char *end = nullptr;
    v = strtof(s, &end);
    return end != s && *end == '\0';
}

// Extracts the next ';'-terminated token from *p, trims whitespace/CR, null-
// terminates into buf, advances *p past the delimiter. Returns false at end
// of string.
bool nextToken(const char **p, char *buf, int bufsize) {
    while (**p == ' ' || **p == '\t' || **p == '\r' || **p == '\n') ++(*p);
    if (**p == '\0') return false;
    int n = 0;
    while (**p != ';' && **p != '\0' && **p != '\r' && **p != '\n' && n < bufsize-1) {
        buf[n++] = **p;
        ++(*p);
    }
    buf[n] = '\0';
    if (**p == ';') ++(*p);
    // trim trailing whitespace
    while (n > 0 && (buf[n-1]==' '||buf[n-1]=='\t')) buf[--n] = '\0';
    return true;
}

float localX(long lon_as, long refLon_as) {
    return (lon_as - refLon_as) * mPerArcsecLon(refLatForLonScale_);
}
float localY(long lat_as, long refLat_as) {
    return (lat_as - refLat_as) * kMPerArcsecLat;
}

bool fail(const char *why, int line) {
    info_.ok = false;
    info_.error = why;
    info_.error_line = line;
    segCount_ = 0;
    activeIdx_ = -1;
    loaded_ = false;
    return false;
}

// Along-track remaining / cross-track / course relative to the straight line
// through (ex,ey) along `course` - used for the final-point hold.
void pointGeometry(float px, float py, float ex, float ey, float course,
                   float &along_to_go, float &cross) {
    const float ux = sinf(course), uy = cosf(course);
    along_to_go = ux*(ex - px) + uy*(ey - py);
    cross       = uy*(px - ex) - ux*(py - ey);
}

}  // namespace

// ---------------------------------------------------------------------------
const MissionInfo &info() { return info_; }
bool loaded() { return loaded_; }
bool complete() { return missionDone_; }
bool hasOrigin() { return haveOrigin_; }

void setOrigin(double lat_deg, double lon_deg) {
    originLatDeg_ = lat_deg;
    originLonDeg_ = lon_deg;
    refLatForLonScale_ = (float)lat_deg;
    haveOrigin_ = true;
}

void gpsToLocal(double lat_deg, double lon_deg, float &x_m, float &y_m) {
    const double dlat_deg = lat_deg - originLatDeg_;
    const double dlon_deg = lon_deg - originLonDeg_;
    y_m = (float)(dlat_deg * 3600.0) * kMPerArcsecLat;
    x_m = (float)(dlon_deg * 3600.0) * mPerArcsecLon(refLatForLonScale_);
}

void localToGps(float x_m, float y_m, double &lat_deg, double &lon_deg) {
    lat_deg = originLatDeg_ + (double)y_m / (kMPerArcsecLat * 3600.0);
    lon_deg = originLonDeg_ + (double)x_m / ((double)mPerArcsecLon(refLatForLonScale_) * 3600.0);
}

bool pathPointAt(float s_m, float &x_m, float &y_m) {
    if (!loaded_ || segCount_ == 0) return false;
    if (s_m < 0.0f) s_m = 0.0f;
    int i = segCount_ - 1;
    while (i > 0 && s_m < cumStart_[i]) --i;
    const Segment &s = segs_[i];
    const float t = fminf(s_m - cumStart_[i], s.length_m);
    if (s.course == Course::LINE) {
        const float f = (s.length_m > 1.0e-3f) ? t / s.length_m : 1.0f;
        x_m = s.start_x + f * (s.target_x - s.start_x);
        y_m = s.start_y + f * (s.target_y - s.start_y);
    } else {
        const float dth = t / s.radius_m;
        const float th  = (s.course == Course::ARC_CW) ? s.theta_start + dth : s.theta_start - dth;
        x_m = s.center_x + s.radius_m * sinf(th);
        y_m = s.center_y + s.radius_m * cosf(th);
    }
    return true;
}

// ---------------------------------------------------------------------------
bool begin(const char *text) {
    segCount_ = 0;
    activeIdx_ = -1;
    missionDone_ = false;
    loaded_ = false;
    arcProgressValid_ = false;
    groundClearanceM_ = 0.0f;
    info_ = MissionInfo{};

    if (text == nullptr) return fail("no mission text", 0);

    long refLat_as = 0, refLon_as = 0;
    bool haveF90 = false, haveF43 = false, haveF39 = false;

    float curX = 0.0f, curY = 0.0f;      // running position while parsing (m)

    const char *p = text;
    char line[128];
    int  lineNo = 0;

    while (*p) {
        // pull one line (up to '\n'); a line that doesn't fit is an error,
        // not something to silently split into two
        int n = 0;
        while (*p && *p != '\n' && n < (int)sizeof(line)-1) line[n++] = *p++;
        line[n] = '\0';
        ++lineNo;
        if (*p != '\n' && *p != '\0') return fail("line too long", lineNo);
        if (*p == '\n') ++p;
        if (n > 0 && line[n-1] == '\r') line[n-1] = '\0';

        const char *lp = line;
        char tok[32];
        if (!nextToken(&lp, tok, sizeof(tok))) continue;   // blank line
        if (tok[0] == '%' || tok[0] == '\0') continue;      // file delimiter

        if (strcmp(tok, "F43") == 0) {
            while (nextToken(&lp, tok, sizeof(tok))) {
                if (tok[0] == 'Z') {
                    if (!parseFloat(tok+1, groundClearanceM_) || !(groundClearanceM_ > 0.0f))
                        return fail("F43: bad Z (ground clearance)", lineNo);
                    haveF43 = true;
                }
            }
            if (!haveF43) return fail("F43 without a Z value", lineNo);
        } else if (strcmp(tok, "F90") == 0) {
            long lon_as = 0, lat_as = 0;
            bool gotX = false, gotY = false;
            while (nextToken(&lp, tok, sizeof(tok))) {
                if (tok[0] == 'X')      gotX = parseDms(tok+1, lon_as);
                else if (tok[0] == 'Y') gotY = parseDms(tok+1, lat_as);
            }
            if (!gotX || !gotY) return fail("F90: bad/missing X or Y (packed DMS, N/E only)", lineNo);
            refLat_as = lat_as; refLon_as = lon_as;
            setOrigin(lat_as / 3600.0, lon_as / 3600.0);
            haveF90 = true;
            curX = 0.0f; curY = 0.0f;
        } else if (strcmp(tok, "F01") == 0) {
            if (!haveF90 || !haveF43) return fail("segment before F43/F90", lineNo);
            if (segCount_ >= kMaxSegments) return fail("too many segments", lineNo);
            float dLonAs = 0.0f, dLatAs = 0.0f;   // F01: X=Dlongitude, Y=Dlatitude
            bool gotX = false, gotY = false;
            while (nextToken(&lp, tok, sizeof(tok))) {
                if (tok[0] == 'X')      gotX = parseFloat(tok+1, dLonAs);
                else if (tok[0] == 'Y') gotY = parseFloat(tok+1, dLatAs);
            }
            if (!gotX || !gotY) return fail("F01: bad/missing X or Y", lineNo);
            Segment &s = segs_[segCount_++];
            s = Segment{};
            s.course = Course::LINE;
            s.start_x = curX; s.start_y = curY;
            s.target_x = curX + dLonAs * mPerArcsecLon(refLatForLonScale_);
            s.target_y = curY + dLatAs * kMPerArcsecLat;
            const float dx = s.target_x - s.start_x, dy = s.target_y - s.start_y;
            s.length_m = hypotf(dx, dy);
            s.end_course_rad = atan2f(dx, dy);
            curX = s.target_x; curY = s.target_y;
        } else if (strcmp(tok, "F02") == 0 || strcmp(tok, "F03") == 0) {
            if (!haveF90 || !haveF43) return fail("segment before F43/F90", lineNo);
            if (segCount_ >= kMaxSegments) return fail("too many segments", lineNo);
            const bool isCw = (tok[2] == '2');   // capture BEFORE tok gets reused below
            long cLon_as = 0, cLat_as = 0, eLon_as = 0, eLat_as = 0;
            bool gotI = false, gotJ = false, gotX = false, gotY = false;
            bool gotR = false, gotA = false;
            float fileR = 0.0f, fileA = 0.0f;
            while (nextToken(&lp, tok, sizeof(tok))) {
                if      (tok[0] == 'I') gotI = parseDms(tok+1, cLon_as);
                else if (tok[0] == 'J') gotJ = parseDms(tok+1, cLat_as);
                else if (tok[0] == 'X') gotX = parseDms(tok+1, eLon_as);
                else if (tok[0] == 'Y') gotY = parseDms(tok+1, eLat_as);
                else if (tok[0] == 'R') gotR = parseFloat(tok+1, fileR);
                else if (tok[0] == 'A') gotA = parseFloat(tok+1, fileA);
            }
            if (!gotI || !gotJ || !gotX || !gotY)
                return fail("arc: bad/missing I, J, X or Y (packed DMS, N/E only)", lineNo);

            Segment &s = segs_[segCount_++];
            s = Segment{};
            s.course = isCw ? Course::ARC_CW : Course::ARC_CCW;
            s.start_x = curX; s.start_y = curY;
            s.center_x = localX(cLon_as, refLon_as); s.center_y = localY(cLat_as, refLat_as);
            s.target_x = localX(eLon_as, refLon_as); s.target_y = localY(eLat_as, refLat_as);
            s.radius_m = hypotf(s.start_x - s.center_x, s.start_y - s.center_y);
            if (s.radius_m < 1.0f) return fail("arc: start point coincides with center", lineNo);

            // Bearings from the center are compass angles: they INCREASE for
            // CW travel and DECREASE for CCW travel.
            s.theta_start = atan2f(s.start_x - s.center_x, s.start_y - s.center_y);
            const float theta_end = atan2f(s.target_x - s.center_x, s.target_y - s.center_y);
            s.sweep_rad = isCw ? wrap2Pi(theta_end - s.theta_start)
                               : wrap2Pi(s.theta_start - theta_end);
            if (s.sweep_rad < 1.0e-4f) s.sweep_rad = kTwoPi;   // start == end: full circle
            s.length_m = s.radius_m * s.sweep_rad;
            s.end_course_rad = wrapPi(isCw ? theta_end + 0.5f*kPi : theta_end - 0.5f*kPi);

            // Sanity warnings (never used for guidance - see header).
            const float r_end = hypotf(s.target_x - s.center_x, s.target_y - s.center_y);
            bool warn = fabsf(r_end - s.radius_m) > 0.01f * kArcRadiusTolPct * s.radius_m;
            if (gotR && fileR > 0.0f) {
                const float pct = 100.0f * fabsf(fileR - s.radius_m) / s.radius_m;
                if (pct > info_.worst_arc_radius_err_pct) info_.worst_arc_radius_err_pct = pct;
                if (pct > kArcRadiusTolPct) warn = true;
            }
            if (gotA) {
                const float err = fabsf(fabsf(fileA) - s.sweep_rad * 180.0f / kPi);
                if (err > info_.worst_arc_sweep_err_deg) info_.worst_arc_sweep_err_deg = err;
                if (err > kArcSweepTolDeg) warn = true;
            }
            if (warn) ++info_.arc_warnings;

            curX = s.target_x; curY = s.target_y;
        } else if (strcmp(tok, "F39") == 0) {
            haveF39 = true;
            break;
        } else {
            ++info_.unknown_lines;   // not path-defining - ignored
        }
    }

    if (!haveF43)       return fail("missing F43 (mission start / ground clearance)", 0);
    if (!haveF90)       return fail("missing F90 (absolute start)", 0);
    if (!haveF39)       return fail("missing F39 (mission end) - file truncated?", 0);
    if (segCount_ == 0) return fail("no path segments", 0);

    float after = 0.0f;
    for (int i = segCount_ - 1; i >= 0; --i) {
        remainingAfter_[i] = after;
        after += segs_[i].length_m;
    }
    float before = 0.0f;
    for (int i = 0; i < segCount_; ++i) {
        cumStart_[i] = before;
        before += segs_[i].length_m;
    }

    info_.ok                 = true;
    info_.segment_count      = segCount_;
    info_.total_length_m     = after;
    info_.ground_clearance_m = groundClearanceM_;
    info_.origin_lat_deg     = originLatDeg_;
    info_.origin_lon_deg     = originLonDeg_;

    activeIdx_ = 0;
    loaded_ = true;
    return true;
}

// ---------------------------------------------------------------------------
void update(float pos_x_m, float pos_y_m, NavOutput &out) {
    out.segment_completed  = false;
    out.ground_clearance_m = groundClearanceM_;

    if (!loaded_ || segCount_ == 0) {
        out = NavOutput{};
        return;
    }

    if (missionDone_) {
        const Segment &last = segs_[segCount_ - 1];
        float along, cross;
        pointGeometry(pos_x_m, pos_y_m, last.target_x, last.target_y, last.end_course_rad,
                      along, cross);
        out.en_route               = false;
        out.mission_complete       = true;
        out.course                 = last.course;
        out.segment_index          = segCount_ - 1;
        out.desired_course_rad     = last.end_course_rad;
        out.cross_track_error      = cross;
        out.distance_to_go         = along;
        out.mission_distance_to_go = along;
        return;
    }

    const Segment &s = segs_[activeIdx_];
    out.en_route         = true;
    out.mission_complete = false;
    out.course           = s.course;
    out.segment_index    = activeIdx_;

    float remaining;

    if (s.course == Course::LINE) {
        const float dx = s.target_x - s.start_x;
        const float dy = s.target_y - s.start_y;
        float ux = 0.0f, uy = 1.0f;
        if (s.length_m > 1.0e-3f) { ux = dx / s.length_m; uy = dy / s.length_m; }

        out.desired_course_rad = s.end_course_rad;   // constant along a line

        const float rx = pos_x_m - s.start_x;
        const float ry = pos_y_m - s.start_y;
        out.cross_track_error = uy*rx - ux*ry;       // right of travel positive
        remaining = s.length_m - (ux*rx + uy*ry);    // < 0 once past the end

#ifdef FCODE_DEBUG
        printf("[DEBUG LINE] start=(%.2f,%.2f) target=(%.2f,%.2f) len=%.2f rem=%.2f\n",
            s.start_x, s.start_y, s.target_x, s.target_y, s.length_m, remaining);
#endif

    } else {  // ARC_CW or ARC_CCW
        const bool ccw = (s.course == Course::ARC_CCW);
        const float theta_now = atan2f(pos_x_m - s.center_x, pos_y_m - s.center_y);
        const float r_now = hypotf(pos_x_m - s.center_x, pos_y_m - s.center_y);
        const float radial_error = r_now - s.radius_m;

        // CCW: center on the LEFT, so too far OUT (radial_error>0) = too far
        // RIGHT (cross_track_error>0). CW: center on the RIGHT, so too far
        // out = too far LEFT (opposite sign).
        out.cross_track_error  = ccw ? radial_error : -radial_error;
        out.desired_course_rad = wrapPi(ccw ? (theta_now - 0.5f*kPi) : (theta_now + 0.5f*kPi));

        // Swept angle from the arc start, in the direction of travel.
        const float phase = ccw ? wrap2Pi(s.theta_start - theta_now)
                                : wrap2Pi(theta_now - s.theta_start);
        if (!arcProgressValid_) {
            // First cycle on this arc: split the unswept gap at its midpoint -
            // positions in its second half are "before the start", not "past
            // the end". A full circle has no gap: use +/-180 deg around the start.
            const float gap    = kTwoPi - s.sweep_rad;
            const float behind = (gap < 1.0e-3f) ? kPi : (s.sweep_rad + 0.5f*gap);
            arcProgress_  = (phase > behind) ? phase - kTwoPi : phase;
            arcProgressValid_ = true;
        } else {
            arcProgress_ += wrapPi(phase - arcPhasePrev_);
        }
        arcPhasePrev_ = phase;

        remaining = s.radius_m * (s.sweep_rad - arcProgress_);

#ifdef FCODE_DEBUG
        printf("[DEBUG ARC] center=(%.2f,%.2f) r=%.2f sweep=%.2fdeg progress=%.2fdeg rem=%.2f\n",
            s.center_x, s.center_y, s.radius_m, s.sweep_rad*57.2958f,
            arcProgress_*57.2958f, remaining);
#endif
    }

    out.distance_to_go         = remaining;
    out.mission_distance_to_go = fmaxf(remaining, 0.0f) + remainingAfter_[activeIdx_];

    if (remaining <= kCompletionM) {
        out.segment_completed = true;
        arcProgressValid_ = false;
        ++activeIdx_;
        if (activeIdx_ >= segCount_) {
            activeIdx_   = segCount_ - 1;
            missionDone_ = true;
            out.en_route         = false;
            out.mission_complete = true;
        }
    }
}

}  // namespace fcode

// ---------------------------------------------------------------------------
// loadFCodeFromSd - separated behind ARDUINO so the parser/guidance math
// above stays host-testable (no SD/SPI hardware) exactly like
// flight_kinematics.cpp does. Uses the standard Arduino `SD` library over
// SPI with the default SPI pins and the given CS pin - if your card is wired
// differently (SDMMC 4-bit mode, a non-default SPI bus), this is the only
// function that needs to change; nothing else in this file touches hardware.
// ---------------------------------------------------------------------------
#ifdef ARDUINO
#include <SD.h>
#include <SPI.h>

namespace fcode {

static char sdBuf_[8192];   // generous for any realistic mission file; a
                             // multi-hundred-segment mission is still only a
                             // few KB of ASCII F-Code
static const char *loadError_ = nullptr;

const char *loadError() { return loadError_; }

const char *loadFCodeFromSd(const char *path, uint8_t csPin) {
    loadError_ = nullptr;
    if (!SD.begin(csPin)) { loadError_ = "SD card init failed"; return nullptr; }
    File f = SD.open(path, FILE_READ);
    if (!f) { loadError_ = "mission file not found"; return nullptr; }
    const size_t size = f.size();
    if (size >= sizeof(sdBuf_)) {
        f.close();
        loadError_ = "mission file larger than the 8 KB buffer";
        return nullptr;
    }
    const size_t n = f.readBytes(sdBuf_, size);
    f.close();
    if (n != size) { loadError_ = "short read from SD"; return nullptr; }
    sdBuf_[n] = '\0';
    return sdBuf_;
}

}  // namespace fcode
#endif  // ARDUINO
