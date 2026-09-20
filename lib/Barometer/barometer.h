#pragma once
#include "I2C_IO.h"
#define BMP280_ADDRESS 0x76 // SDO to GND
#define BMP280_OUTPUT_REG 0xF7

struct calibData280 {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
};

struct bmpReadingsInt32 {
    int32_t pressure;
    int32_t temperature;
};

struct bmpReadingsDouble {
    double pressure;
    double temperature;
    double altitude;
};


extern calibData280 BMP280calib;

extern bmpReadingsInt32 bmp_raw;

extern bmpReadingsDouble bmp_out;

extern SemaphoreHandle_t i2cMutex;

void readCalibrationData280(calibData280 &calib);

void printCalibration280(const calibData280 &calib);

void initBMP280();

bool readBMP280(const calibData280 &calib, bmpReadingsInt32 &raw_readings, bmpReadingsDouble &output);

void printPressTemp(const bmpReadingsDouble &outvals);