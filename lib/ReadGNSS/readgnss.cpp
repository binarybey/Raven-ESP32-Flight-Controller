#include "readgnss.h"
#include "nmea_parse.h"
#include <string.h>

HardwareSerial GNSS(GNSS_SERIAL_PORT);

GNSSData masterGNSSData;
SatelliteData masterSatellites[MAX_SATELLITES];
volatile uint32_t nmeaChecksumErrors = 0;

void beginGNSS() {
    GNSS.setRxBufferSize(1024);   // must precede begin()
    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
    GNSS.setTimeout(10);
}

// Publishes one parsed sentence. Returns true for an accepted GGA.
static bool publish(const nmea::Sentence &s) {
    static SatelliteData tempSatellites[MAX_SATELLITES];
    static byte tempVisible = 0;

    if (s.type == nmea::Type::GGA) {
        const nmea::Gga &g = s.gga;
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
        GNSSData &d = masterGNSSData;
        d.time_Hours = g.hour; d.time_Minutes = g.minute; d.time_Seconds = g.second;
        d.fixQuality = g.fix_quality;
        d.satellite_number_active = g.sats_used;
        d.HDOP = g.hdop;
        if (g.fix_quality > 0) {
            d.latitude_deg  = g.lat.deg;
            d.longitude_deg = g.lon.deg;
            d.latitude_Degrees  = g.lat.d; d.latitude_Minutes  = g.lat.m;
            d.latitude_Seconds  = g.lat.s; d.latitude_Direction  = g.lat.hemi;
            d.longitude_Degrees = g.lon.d; d.longitude_Minutes = g.lon.m;
            d.longitude_Seconds = g.lon.s; d.longitude_Direction = g.lon.hemi;
            d.true_Altitude = g.altitude_msl_m;
        }
        d.fixSeq++;
        d.fixMillis = millis();
        xSemaphoreGive(gnssMutex);
        return true;
    }

    if (s.type == nmea::Type::RMC) {
        const nmea::Rmc &r = s.rmc;
        if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(5)) != pdTRUE) return false;
        GNSSData &d = masterGNSSData;
        if (r.day != 0) { d.time_Day = r.day; d.time_Month = r.month; d.time_Year = r.year; }
        d.rmcValid       = r.active;
        d.ground_Speed   = r.speed_knots;
        d.courseValid    = r.has_course;
        d.course_Degrees = r.course_deg;
        d.velSeq++;
        d.velMillis = millis();
        xSemaphoreGive(gnssMutex);
        return false;
    }

    // GSV: GPS satellites only (GPGSV) - other constellations number their
    // own GSV groups from 1, and mixing them would scramble the list.
    if (s.type == nmea::Type::GSV && s.talker[0] == 'G' && s.talker[1] == 'P') {
        const nmea::Gsv &v = s.gsv;
        if (v.msg_num == 1) {
            memset(tempSatellites, 0, sizeof(tempSatellites));
        }
        tempVisible = v.sats_in_view;
        for (int k = 0; k < v.count; ++k) {
            const int idx = (v.msg_num - 1) * 4 + k;
            if (idx >= MAX_SATELLITES) break;
            tempSatellites[idx].satelliteID = v.sats[k].id;
            tempSatellites[idx].elevation   = v.sats[k].elevation;
            tempSatellites[idx].azimuth     = v.sats[k].azimuth;
            tempSatellites[idx].SNR         = v.sats[k].snr;
        }
        if (v.msg_num == v.total_msgs &&
            xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            memcpy(masterSatellites, tempSatellites, sizeof(tempSatellites));
            masterGNSSData.satellite_number_visible = tempVisible;
            xSemaphoreGive(gnssMutex);
        }
    }
    return false;
}

bool readNMEA(HardwareSerial* GNSS) {
    static char line[100];
    static int  n = 0;
    static bool inSentence = false;

    bool newFixAvailable = false;

    while (GNSS->available() > 0) {
        const char c = GNSS->read();

        if (c == '$') {                 // start of a sentence (resyncs on garbage)
            inSentence = true;
            n = 0;
            continue;
        }
        if (!inSentence) continue;

        if (c == '\r' || c == '\n') {
            inSentence = false;
            line[n] = '\0';
            nmea::Sentence s;
            const nmea::Result r = nmea::parse(line, s);
            if (r == nmea::Result::OK) {
                if (publish(s)) newFixAvailable = true;
            } else if (r == nmea::Result::BAD_CHECKSUM || r == nmea::Result::MALFORMED) {
                nmeaChecksumErrors = nmeaChecksumErrors + 1;
            }
            continue;
        }

        if (n < (int)sizeof(line) - 1) line[n++] = c;
        else inSentence = false;        // overlong: drop it, wait for the next '$'
    }
    return newFixAvailable;
}

void printGNSS(){
    GNSSData d;
    SatelliteData sats[MAX_SATELLITES];
    if (xSemaphoreTake(gnssMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;
    d = masterGNSSData;
    memcpy(sats, masterSatellites, sizeof(sats));
    xSemaphoreGive(gnssMutex);

    Serial.println("====== GNSS TELEMETRY ======");

    // Time and date, %02d forces leading zeros
    Serial.printf("Time: %02d:%02d:%02d | Date: %02d/%02d/%02d\n",
        d.time_Hours, d.time_Minutes, d.time_Seconds,
        d.time_Day, d.time_Month, d.time_Year);

    Serial.printf("Lat: %d deg %d min %.4f sec %c  (%.7f)\n",
        d.latitude_Degrees, d.latitude_Minutes, d.latitude_Seconds, d.latitude_Direction,
        d.latitude_deg);
    Serial.printf("Lon: %d deg %d min %.4f sec %c  (%.7f)\n",
        d.longitude_Degrees, d.longitude_Minutes, d.longitude_Seconds, d.longitude_Direction,
        d.longitude_deg);

    Serial.printf("Fix: %d | Sats used/visible: %d/%d | HDOP: %.2f | Alt: %.1fm | Spd: %.2f knots | Course: %s%.1f | NMEA errors: %lu\n",
        d.fixQuality, d.satellite_number_active, d.satellite_number_visible,
        d.HDOP, d.true_Altitude, d.ground_Speed,
        d.courseValid ? "" : "(invalid) ", d.course_Degrees,
        (unsigned long)nmeaChecksumErrors);

    int visible = d.satellite_number_visible;
    if (visible > MAX_SATELLITES) visible = MAX_SATELLITES;

    Serial.println("--- Visible GPS Satellites ---");
    for (int i = 0; i < visible; i++) {
        if (sats[i].satelliteID != 0) {
            Serial.printf("ID: %02d | Elev: %02d | Azim: %03d | SNR: %02d\n",
                sats[i].satelliteID, sats[i].elevation, sats[i].azimuth, sats[i].SNR);
        }
    }
    Serial.println("==========================\n");
}
