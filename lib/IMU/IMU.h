#pragma once
#include "I2C_IO.h"

#define MAG_ADDRESS 0x1E
#define MAG_REG_A 0x00
#define MAG_REG_B 0x01
#define MAG_MD_REG 0x02
#define MAG_SAMPLE_AVG_1 0b00
#define MAG_SAMPLE_AVG_2 0b01
#define MAG_SAMPLE_AVG_4 0b10
#define MAG_SAMPLE_AVG_8 0b11
#define MAG_OUTPUT_RATE_0_75 0b000
#define MAG_OUTPUT_RATE_1_5 0b001
#define MAG_OUTPUT_RATE_3 0b010
#define MAG_OUTPUT_RATE_7_5 0b011
#define MAG_OUTPUT_RATE_15 0b100
#define MAG_OUTPUT_RATE_30 0b101
#define MAG_OUTPUT_RATE_75 0b110
#define MAG_SENS_1370 0b000 // LSB per Gauss
#define MAG_SENS_1090 0b001
#define MAG_SENS_820 0b010
#define MAG_SENS_660 0b011
#define MAG_SENS_440 0b100
#define MAG_SENS_390 0b101
#define MAG_SENS_330 0b110
#define MAG_SENS_230 0b111
#define MAG_SINGLE_MEAS 0b01
#define MAG_CONTINUOUS_MEAS 0b00
#define MAG_OUTPUT_REG 0x03

#define MPU_ADDRESS 0x68
#define MPU_PMGMT1 0x6B
#define MPU_IIC_BYPASS 0x37
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_DLPF_0 0b000
#define MPU_DLPF_1 0b001
#define MPU_DLPF_2 0b010
#define MPU_DLPF_3 0b011
#define MPU_DLPF_4 0b100
#define MPU_DLPF_5 0b101
#define MPU_DLPF_6 0b110
#define MPU_GYRO_FS_250 0b00    // values are plus-minus in range
#define MPU_GYRO_FS_500 0b01
#define MPU_GYRO_FS_1000 0b10
#define MPU_GYRO_FS_2000 0b11
#define MPU_ACCEL_FS_2 0b00     // values are plus-minus in range
#define MPU_ACCEL_FS_4 0b01
#define MPU_ACCEL_FS_8 0b10
#define MPU_ACCEL_FS_16 0b11
#define MPU_OUTPUT_REG 0x3B


struct D3_int {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct D3 {
    float x;
    float y;
    float z;
};

struct MPU_output
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t mputemp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
};


extern D3 accel;
extern D3 gyro;
extern D3 mag;
extern D3 accelC;
extern D3 gyroC;
extern D3 magC;
extern D3 gyroOffset;
extern float mpuTemp;

uint8_t magConfRegA(const int MA, const int DO);
uint8_t magConfRegB(const int GN);
uint8_t magModeReg(const int MD);

void setMag(const uint8_t rega, const uint8_t regb, const uint8_t md);

void readMag(D3 &magbuffer);

void printUncalibratedMag(const D3 &magt);

void applyMagCalibration(const D3 &magt, D3 &magCalibrated);

// ================================================================== //

uint8_t setMPU_DLPF(const int DLPF_CFG);

uint8_t setGyroFS(const int fs_sel);

uint8_t setAccFS(const int afs_sel);

void setMPU(const uint8_t DLPF, const uint8_t FS_SEL, const uint8_t AFS_SEL);

void readMPU(D3 &accbuffer, D3 &gyrobuffer);

void printUncalibratedAccel(const D3 &acct);

void applyAccelCalibration(const D3 &accelt, D3 &accelCalibrated);

void measureGyroOffset(D3 &accelt, D3 &gyrot, D3 &gyroOffsetTemp);

void calibrateGyro(D3 &gyrot, const D3 &gyrotOffset, D3 &gyroCalibrated);

void printIMU(const D3 &acct, const D3 &gyrot, const D3 &magt);