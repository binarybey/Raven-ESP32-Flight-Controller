#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#define GNSS_SERIAL_PORT 2
#define MAX_SATELLITES 24

struct GNSSData {
    byte time_Day;
    byte time_Month;
    byte time_Year;
    byte time_Hours;
    byte time_Minutes;
    byte time_Seconds;
    byte latitude_Degrees;
    byte latitude_Minutes;
    float latitude_Seconds;
    char latitude_Direction;
    byte longitude_Degrees;
    byte longitude_Minutes;
    float longitude_Seconds;
    char longitude_Direction;
    byte fixQuality;
    byte satellite_number_active;
    byte satellite_number_visible;
    float HDOP;
    float true_Altitude;
};

struct SatelliteData {
    byte satelliteID;
    byte elevation;
    int azimuth;
    byte SNR;
};

extern GNSSData masterGNSSData;
extern SatelliteData masterSatellites[MAX_SATELLITES];

extern SemaphoreHandle_t gnssMutex;

bool readNMEA(HardwareSerial* GNSS);

void printGNSS();