// nmea_parse.cpp - see nmea_parse.h.

#include "nmea_parse.h"

#include <string.h>
#include <stdlib.h>

namespace nmea {

namespace {

constexpr int kMaxFields = 24;
constexpr int kMaxBody   = 100;   // NMEA caps sentences at 82 chars

int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

bool allDigits(const char *s, int n) {
    for (int i = 0; i < n; ++i)
        if (s[i] < '0' || s[i] > '9') return false;
    return true;
}

// "hhmmss" or "hhmmss.ss"
bool parseTime(const char *s, uint8_t &h, uint8_t &m, uint8_t &sec) {
    if (strlen(s) < 6 || !allDigits(s, 6)) return false;
    h   = (uint8_t)((s[0]-'0')*10 + (s[1]-'0'));
    m   = (uint8_t)((s[2]-'0')*10 + (s[3]-'0'));
    sec = (uint8_t)((s[4]-'0')*10 + (s[5]-'0'));
    return h < 24 && m < 60 && sec < 61;
}

// "ddmmyy"
bool parseDate(const char *s, uint8_t &d, uint8_t &mo, uint8_t &y) {
    if (strlen(s) != 6 || !allDigits(s, 6)) return false;
    d  = (uint8_t)((s[0]-'0')*10 + (s[1]-'0'));
    mo = (uint8_t)((s[2]-'0')*10 + (s[3]-'0'));
    y  = (uint8_t)((s[4]-'0')*10 + (s[5]-'0'));
    return true;
}

bool parseNumber(const char *s, double &v) {
    if (!s[0]) return false;
    char *end = nullptr;
    v = strtod(s, &end);
    return end != s && *end == '\0';
}

// "ddmm.mmmm..." (degDigits=2) or "dddmm.mmmm..." (degDigits=3), any number
// of minute decimals.
bool parseLatLon(const char *s, const char *hemi, int degDigits, char pos, char neg,
                 LatLon &out) {
    out = LatLon{};
    if (!s[0] || !hemi[0]) return false;
    const char *dot = strchr(s, '.');
    const int intLen = dot ? (int)(dot - s) : (int)strlen(s);
    if (intLen != degDigits + 2 || !allDigits(s, intLen)) return false;
    int deg = 0;
    for (int i = 0; i < degDigits; ++i) deg = deg*10 + (s[i]-'0');
    double minutes;
    if (!parseNumber(s + degDigits, minutes) || minutes < 0.0 || minutes >= 60.0) return false;
    if (hemi[0] != pos && hemi[0] != neg) return false;

    double v = deg + minutes / 60.0;
    if (v > (degDigits == 2 ? 90.0 : 180.0)) return false;
    out.valid = true;
    out.deg   = (hemi[0] == neg) ? -v : v;
    out.d     = (uint8_t)deg;
    out.m     = (uint8_t)minutes;
    out.s     = (float)((minutes - out.m) * 60.0);
    out.hemi  = hemi[0];
    return true;
}

}  // namespace

Result parse(const char *body, Sentence &out) {
    out = Sentence{};

    // --- checksum: XOR of every char between '$' and '*' ---
    const char *star = strchr(body, '*');
    if (!star) return Result::BAD_CHECKSUM;
    const int len = (int)(star - body);
    if (len <= 0 || len >= kMaxBody) return Result::MALFORMED;
    uint8_t sum = 0;
    for (int i = 0; i < len; ++i) sum ^= (uint8_t)body[i];
    const int hi = hexVal(star[1]);
    const int lo = (hi < 0) ? -1 : hexVal(star[2]);
    if (hi < 0 || lo < 0 || (uint8_t)((hi << 4) | lo) != sum) return Result::BAD_CHECKSUM;

    // --- split into fields (in a local copy; empty fields preserved) ---
    char buf[kMaxBody];
    memcpy(buf, body, len);
    buf[len] = '\0';
    const char *f[kMaxFields];
    int nf = 0;
    f[nf++] = buf;
    for (char *c = buf; *c; ++c) {
        if (*c == ',') {
            *c = '\0';
            if (nf < kMaxFields) f[nf++] = c + 1;
        }
    }
    for (int i = nf; i < kMaxFields; ++i) f[i] = "";   // missing trailing fields read as empty

    // --- address: 2-char talker + 3-char type ("GNGGA"). Proprietary
    //     sentences ("PUBX", ...) have a different shape and are ignored. ---
    if (strlen(f[0]) != 5) return Result::IGNORED;
    out.talker[0] = f[0][0];
    out.talker[1] = f[0][1];
    const char *type = f[0] + 2;

    if (strcmp(type, "GGA") == 0) {
        // 1 time, 2 lat, 3 N/S, 4 lon, 5 E/W, 6 quality, 7 sats, 8 hdop, 9 alt, 10 'M'
        Gga &g = out.gga;
        out.type = Type::GGA;
        if (f[1][0] && !parseTime(f[1], g.hour, g.minute, g.second)) return Result::MALFORMED;
        g.fix_quality = (uint8_t)atoi(f[6]);
        g.sats_used   = (uint8_t)atoi(f[7]);
        double v;
        if (parseNumber(f[8], v)) g.hdop = (float)v;
        if (parseNumber(f[9], v)) { g.altitude_msl_m = (float)v; g.has_altitude = true; }
        if (g.fix_quality > 0) {
            if (!parseLatLon(f[2], f[3], 2, 'N', 'S', g.lat) ||
                !parseLatLon(f[4], f[5], 3, 'E', 'W', g.lon) || !g.has_altitude)
                return Result::MALFORMED;
        }
        return Result::OK;
    }

    if (strcmp(type, "RMC") == 0) {
        // 1 time, 2 status, 3-6 lat/lon, 7 speed kn, 8 course, 9 date
        Rmc &r = out.rmc;
        out.type = Type::RMC;
        if (f[1][0] && !parseTime(f[1], r.hour, r.minute, r.second)) return Result::MALFORMED;
        r.active = (f[2][0] == 'A');
        double v;
        if (parseNumber(f[7], v)) r.speed_knots = (float)v;
        else if (r.active) return Result::MALFORMED;
        if (parseNumber(f[8], v)) { r.course_deg = (float)v; r.has_course = true; }
        if (f[9][0] && !parseDate(f[9], r.day, r.month, r.year)) return Result::MALFORMED;
        return Result::OK;
    }

    if (strcmp(type, "GSV") == 0) {
        // 1 total msgs, 2 msg num, 3 sats in view, then 4 x (id, elev, azim, snr)
        Gsv &s = out.gsv;
        out.type = Type::GSV;
        s.total_msgs   = (uint8_t)atoi(f[1]);
        s.msg_num      = (uint8_t)atoi(f[2]);
        s.sats_in_view = (uint8_t)atoi(f[3]);
        if (s.total_msgs == 0 || s.msg_num == 0 || s.msg_num > s.total_msgs) return Result::MALFORMED;
        for (int k = 0; k < 4; ++k) {
            const int base = 4 + 4*k;
            if (base >= nf || !f[base][0]) break;
            GsvSat &sat = s.sats[s.count++];
            sat.id        = (uint8_t)atoi(f[base]);
            sat.elevation = (uint8_t)atoi(f[base+1]);
            sat.azimuth   = (uint16_t)atoi(f[base+2]);
            sat.snr       = (uint8_t)atoi(f[base+3]);
        }
        return Result::OK;
    }

    return Result::IGNORED;
}

}  // namespace nmea
