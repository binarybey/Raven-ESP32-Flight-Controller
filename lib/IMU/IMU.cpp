#include <IMU.h>

D3 accel;
D3 gyro;
D3 mag;
D3 accelC;
D3 gyroC;
D3 magC;
D3 gyroOffset;
float mpuTemp;

uint8_t magConfRegA(const int MA, const int DO) {
    uint8_t regA = 0b00000000;
    regA = MA << 5 | DO << 2;
    return regA;
}

uint8_t magConfRegB(const int GN) {
    uint8_t regB = 0b00000000;
    regB = GN << 5;
    return regB;
}

uint8_t magModeReg(const int MD) {
    uint8_t md = 0b00000000;
    md = MD;
    return md;
}

void setMag(const uint8_t rega, const uint8_t regb, const uint8_t md) {
    I2C_IO::write_IIC(MAG_ADDRESS, MAG_REG_A, rega);
    I2C_IO::write_IIC(MAG_ADDRESS, MAG_REG_B, regb);
    I2C_IO::write_IIC(MAG_ADDRESS, MAG_MD_REG, md);
}

// usage setMag(magConfRegA(MAG_SAMPLE_AVG_8, MAG_OUTPUT_RATE_75), magConfRegB(MAG_SENS_1370), magModeReg(MAG_CONTINUOUS_MEAS))

uint8_t setMPU_DLPF(const int DLPF_CFG) {
    uint8_t CONFIG = 0b00000000;
    CONFIG = DLPF_CFG;
    return CONFIG;
}

uint8_t setGyroFS(const int fs_sel) {
    uint8_t gyro_fs = 0b00000000;
    gyro_fs = fs_sel << 3;
    return gyro_fs;
}

uint8_t setAccFS(const int afs_sel) {
    uint8_t acc_fs = 0b00000000;
    acc_fs = afs_sel << 3;
    return acc_fs;
}

void setMPU(const uint8_t DLPF, const uint8_t FS_SEL, const uint8_t AFS_SEL) {
    I2C_IO::write_IIC(MPU_ADDRESS, MPU_PMGMT1, 0x00);
    I2C_IO::write_IIC(MPU_ADDRESS, MPU_IIC_BYPASS, 0x02);
    I2C_IO::write_IIC(MPU_ADDRESS, MPU_CONFIG, DLPF);
    I2C_IO::write_IIC(MPU_ADDRESS, MPU_GYRO_CONFIG, FS_SEL);
    I2C_IO::write_IIC(MPU_ADDRESS, MPU_ACCEL_CONFIG, AFS_SEL);
}

// usage setMPU(setMPU_DLPF(MPU_DLPF_2), setGyroFS(MPU_GYRO_FS_250), setAccFS(MPU_ACCEL_FS_2))

bool readMag(D3 &magbuffer) {
    static D3_int mag_raw;
    if (!I2C_IO::read_IIC(MAG_ADDRESS, MAG_OUTPUT_REG, 3, (int16_t*)&mag_raw, true)) return false;
    // HMC5883L reports -4096 on an axis whose ADC overflowed - not a usable sample.
    if (mag_raw.x == MAG_OVERFLOW || mag_raw.y == MAG_OVERFLOW || mag_raw.z == MAG_OVERFLOW) return false;
    magbuffer.x = static_cast<float>(mag_raw.x) * 0.73f;    // convert to miliGauss, saturation occurs when -2989 is reported.
    magbuffer.y = static_cast<float>(mag_raw.z) * 0.73f;
    magbuffer.z = static_cast<float>(mag_raw.y) * 0.73f;
    return true;
}
//  usage readMag(mag)

void printUncalibratedMag(const D3 &magt) {
    Serial.printf("%f,%f,%f\n",magt.x, magt.y, magt.z);
}

void applyMagCalibration(const D3 &magt, D3 &magCalibrated) {
    float tempX = magt.x - (-187.279956f);
    float tempY = magt.y - (7.723305f);
    float tempZ = magt.z - (-0.057187f);

    magCalibrated.x = (0.875656f * tempX) + (0.027110f * tempY) + (-0.054459f * tempZ);
    magCalibrated.y = (0.027110f * tempX) + (0.891379f * tempY) + (0.014806f * tempZ);
    magCalibrated.z = (-0.054459f * tempX) + (0.014806f * tempY) + (0.963926f * tempZ);
}

bool readMPU(D3 &accbuffer, D3 &gyrobuffer) {
    static MPU_output mpudata;
    if (!I2C_IO::read_IIC(MPU_ADDRESS, MPU_OUTPUT_REG, 7, (int16_t*)&mpudata, true)) return false;
    accbuffer.x = (static_cast<float>(mpudata.ax) * 0.000001f) * 61.035156f;
    accbuffer.y = (static_cast<float>(mpudata.ay) * 0.000001f) * 61.035156f;
    accbuffer.z = (static_cast<float>(mpudata.az) * 0.000001f) * 61.035156f;
    mpuTemp = ((static_cast<float>(mpudata.mputemp) * 0.0001f) * 29.411764f) + 36.53f;
    gyrobuffer.x = (static_cast<float>(mpudata.gx) * 0.001f) * 7.633588f;
    gyrobuffer.y = (static_cast<float>(mpudata.gy) * 0.001f) * 7.633588f;
    gyrobuffer.z = (static_cast<float>(mpudata.gz) * 0.001f) * 7.633588f;
    return true;
}
//  usage readMPU(accel, gyro)

void applyAccelCalibration(const D3 &accelt, D3 &accelCalibrated) {
    float tempX = accelt.x - (0.019400f);
    float tempY = accelt.y - (-0.008463f);
    float tempZ = accelt.z - (-0.098649f);

    accelCalibrated.x = (0.991896f * tempX) + (0.000922f * tempY) + (-0.002850f * tempZ);
    accelCalibrated.y = (0.000922f * tempX) + (1.004652f * tempY) + (-0.002416f * tempZ);
    accelCalibrated.z = (-0.002850f * tempX) + (-0.002416f * tempY) + (0.984773f * tempZ);
}

bool measureGyroOffset(D3 &accelt, D3 &gyrot, D3 &gyroOffsetTemp){
    double tempX = 0.0;
    double tempY = 0.0;
    double tempZ = 0.0;

    int good = 0;
    for (int i = 0; i < 2000; i++) {
        if (readMPU(accelt, gyrot)) {
            tempX += gyrot.x;
            tempY += gyrot.y;
            tempZ += gyrot.z;
            good++;
        }
        delay(2); 
    }
    if (good == 0) return false;
    gyroOffsetTemp.x = tempX / good;
    gyroOffsetTemp.y = tempY / good;
    gyroOffsetTemp.z = tempZ / good;
    return true;
}
// usage measureGyroOffset(accel, gyro, gyroOffset)

void calibrateGyro(D3 &gyrot, const D3 &gyrotOffset, D3 &gyroCalibrated){
    gyroCalibrated.x = gyrot.x - gyrotOffset.x;
    gyroCalibrated.y = gyrot.y - gyrotOffset.y;
    gyroCalibrated.z = gyrot.z - gyrotOffset.z;
}
// usage calibrateGyro(gyro, gyroOffset, gyroC)

void printIMU(const D3 &acct, const D3 &gyrot, const D3 &magt) {
    Serial.printf("aX: %f\taY: %f\taZ: %f\tgX: %f\tgY: %f\tgZ: %f\tmX: %f\tmY: %f\tmZ: %f\n", acct.x, acct.y, acct.z, gyrot.x, gyrot.y, gyrot.z, magt.x, magt.y, magt.z);
}

//  usage printIMU(accel, gyro, mag, true)

bool readImuSample(ImuSample &sample, int64_t now_us) {
    D3 accRaw, gyroRaw, magRaw;
    const bool imuOk = readMPU(accRaw, gyroRaw);
    if (imuOk) {
        applyAccelCalibration(accRaw, sample.accel);
        calibrateGyro(gyroRaw, gyroOffset, sample.gyro);
        sample.imu_us = now_us;
    }
    if (readMag(magRaw)) {
        applyMagCalibration(magRaw, sample.mag);
        sample.mag_us = now_us;
    }
    return imuOk;
}
//  usage readImuSample(sample, esp_timer_get_time())

void printUncalibratedAccel(const D3 &acct) {
    Serial.printf("%f,%f,%f\n",acct.x, acct.y, acct.z);
}