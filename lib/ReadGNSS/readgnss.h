#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#define GNSS_SERIAL_PORT 2
#define GNSS_RX_PIN 16
#define GNSS_TX_PIN 17
#define GNSS_BAUD 9600
#define MAX_SATELLITES 24

// Position/fix fields come ONLY from GGA and velocity fields ONLY from RMC,
// so each group is internally consistent (same epoch) no matter which order
// the receiver sends the two sentences in.
struct GNSSData {
    // UTC time (GGA/RMC) and date (RMC)
    byte time_Day;
    byte time_Month;
    byte time_Year;
    byte time_Hours;
    byte time_Minutes;
    byte time_Seconds;

    // --- position group (GGA) ---
    byte latitude_Degrees;      // unsigned DMS, for display
    byte latitude_Minutes;
    float latitude_Seconds;
    char latitude_Direction;
    byte longitude_Degrees;
    byte longitude_Minutes;
    float longitude_Seconds;
    char longitude_Direction;
    double latitude_deg;        // signed decimal degrees (N positive) - use these
    double longitude_deg;       // signed decimal degrees (E positive)   for navigation
    byte fixQuality;            // 0 = no fix
    byte satellite_number_active;
    float HDOP;
    float true_Altitude;        // m, MSL
    uint32_t fixSeq;            // +1 per accepted GGA sentence
    uint32_t fixMillis;         // millis() when it was accepted

    // --- velocity group (RMC) ---
    bool rmcValid;              // RMC status 'A'
    float ground_Speed;         // knots, as transmitted
    bool courseValid;           // false when the receiver leaves the field empty
    float course_Degrees;       // true course over ground, 0..360
    uint32_t velSeq;            // +1 per accepted RMC sentence
    uint32_t velMillis;

    // --- satellites (GPGSV, display only) ---
    byte satellite_number_visible;
};

struct SatelliteData {
    byte satelliteID;
    byte elevation;
    int azimuth;
    byte SNR;
};

extern HardwareSerial GNSS;
extern GNSSData masterGNSSData;
extern SatelliteData masterSatellites[MAX_SATELLITES];
extern volatile uint32_t nmeaChecksumErrors;

extern SemaphoreHandle_t gnssMutex;

// Configures the UART (buffer size, pins, baud). Call once in setup().
void beginGNSS();

// Drains the UART and publishes complete, checksum-verified sentences into
// masterGNSSData under gnssMutex. Call at >= 10 Hz (the UART buffer is 1 KB
// and position latency grows with the polling interval). Returns true if at
// least one GGA was accepted during this call.
bool readNMEA(HardwareSerial* GNSS);

// Copies the shared data under the mutex, then prints outside it - never
// holds gnssMutex while waiting on the serial port.
void printGNSS();
