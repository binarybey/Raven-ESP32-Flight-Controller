// nmea_parse.h - pure NMEA-0183 sentence parser (no Arduino, no FreeRTOS),
// so it can be unit-tested on a PC. readgnss.cpp does the UART buffering,
// mutexes and publishing; this file only turns one sentence into numbers.
//
// - Any talker ID is accepted (GP, GN, GL, GA, GB, ...): multi-constellation
//   receivers send GNGGA/GNRMC, and a GP-only parser never sees a fix.
// - The checksum is REQUIRED and verified. A corrupted sentence is rejected,
//   never half-applied.
// - Latitude/longitude minutes are parsed with however many decimals the
//   receiver sends (4, 5, 6 ...), not a fixed digit count.

#pragma once

#include <stdint.h>

namespace nmea {

enum class Type : uint8_t { NONE, GGA, RMC, GSV };

enum class Result : uint8_t {
    OK,             // parsed; `type` says which struct is filled
    IGNORED,        // well-formed, checksum OK, but not a sentence we use
    BAD_CHECKSUM,   // checksum missing or wrong
    MALFORMED       // checksum OK but a field we need doesn't parse
};

struct LatLon {
    bool    valid = false;
    double  deg   = 0.0;     // signed decimal degrees (N/E positive)
    uint8_t d = 0, m = 0;    // unsigned DMS, for display
    float   s = 0.0f;
    char    hemi = ' ';
};

struct Gga {
    uint8_t hour = 0, minute = 0, second = 0;
    LatLon  lat, lon;
    uint8_t fix_quality = 0;       // 0 = no fix
    uint8_t sats_used   = 0;
    float   hdop        = 99.9f;
    bool    has_altitude = false;
    float   altitude_msl_m = 0.0f;
};

struct Rmc {
    uint8_t hour = 0, minute = 0, second = 0;
    uint8_t day = 0, month = 0, year = 0;
    bool    active = false;        // status 'A' (valid) vs 'V' (warning)
    float   speed_knots = 0.0f;
    bool    has_course  = false;   // empty when stationary on many receivers
    float   course_deg  = 0.0f;    // true course over ground
};

struct GsvSat { uint8_t id = 0, elevation = 0; uint16_t azimuth = 0; uint8_t snr = 0; };

struct Gsv {
    uint8_t total_msgs = 0, msg_num = 0, sats_in_view = 0;
    uint8_t count = 0;             // satellites in THIS message (0-4)
    GsvSat  sats[4];
};

struct Sentence {
    Type type = Type::NONE;
    char talker[3] = {0, 0, 0};    // "GP", "GN", ...
    Gga  gga;
    Rmc  rmc;
    Gsv  gsv;
};

// `body` is the text between '$' and the end of line, e.g.
// "GNGGA,123519.00,4807.03800,N,01131.00000,E,1,08,0.9,545.4,M,46.9,M,,*5C".
Result parse(const char *body, Sentence &out);

}  // namespace nmea
