#include "WiFi.h"
#include "Arduino.h"
#include "readgnss.h"
#include "I2C_IO.h"
#include "barometer.h"
#include "IMU.h"
#include "Fusion.h"

FusionAhrs ahrs;
FusionBias bias;

extern HardwareSerial GNSS;

SemaphoreHandle_t gnssMutex;
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t i2cMutex;

void TaskGNSS(void *pvParameters) {
  GNSS.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
    for(;;) {
        if(readNMEA(&GNSS)){
        printGNSS();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 1Hz loop
    }
}

void TaskIMU(void *pvParameters) {
    for(;;) {

        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            readMPU(accel, gyro);
            readMag(mag);
            xSemaphoreGive(i2cMutex);
        }
        if (xSemaphoreTake(imuMutex, portMAX_DELAY) == pdTRUE) {
            applyAccelCalibration(accel, accelC);
            applyMagCalibration(mag, magC);
            calibrateGyro(gyro, gyroOffset, gyroC);
            xSemaphoreGive(imuMutex);
        }
        //printIMU(accelC, gyro, magC);
        //printUncalibratedMag(mag);
        //printUncalibratedAccel(accel);
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz loop
    }
}

void TaskBMP(void *pvParameters) {
    for(;;) {
        //bool bmpsuccess = false;
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            //bmpsuccess = readBMP280(BMP280calib, bmp_raw, bmp_out);
            xSemaphoreGive(i2cMutex);
        }
        //if (bmpsuccess) {
            //printPressTemp(bmp_out);}
        vTaskDelay(500 / portTICK_PERIOD_MS); // 2Hz loop
    }
}

void TaskFlightControl(void *pvParameters) {
    for(;;) {
        if (xSemaphoreTake(imuMutex, 10) == pdTRUE) {
            
            FusionVector gyroscope = { .axis = {gyroC.x, gyroC.y, gyroC.z} };
            FusionVector accelerometer = { .axis = {accelC.x, accelC.y, accelC.z} };
            FusionVector magnetometer = { .axis = {magC.x, magC.y, magC.z} };
            
            xSemaphoreGive(imuMutex);
            
            gyroscope = FusionBiasUpdate(&bias, gyroscope);

            FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);

            FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

            Serial.printf("Roll: %f\tPitch: %f\tYaw: %f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

            // You now have pitch, roll, and yaw ready for motor mixing:
            // euler.angle.pitch
            // euler.angle.roll
            // euler.angle.yaw
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS); 
    }
}

void setup() {

    Serial.begin(115200);

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    btStop();

    Wire.begin();
    Wire.setClock(400000);

    GNSS.setRxBufferSize(1024); 
    GNSS.begin(9600);
    GNSS.setTimeout(10);

    gnssMutex = xSemaphoreCreateMutex();
    imuMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();
    
    readCalibrationData280(BMP280calib);
    printCalibration280(BMP280calib);
    initBMP280();

    setMPU(setMPU_DLPF(MPU_DLPF_2), setGyroFS(MPU_GYRO_FS_250), setAccFS(MPU_ACCEL_FS_2));
    setMag(magConfRegA(MAG_SAMPLE_AVG_8, MAG_OUTPUT_RATE_75), magConfRegB(MAG_SENS_1370), magModeReg(MAG_CONTINUOUS_MEAS));
    measureGyroOffset(accel, gyro, gyroOffset);


    FusionBiasInitialise(&bias);
    FusionAhrsInitialise(&ahrs);

    const FusionAhrsSettings settings = {
        .sampleRate = 100.0f,                  
        .convention = FusionConventionNwu,     
        .gain = 0.5f,   // Default filter gain
        .gyroscopeRange = 250.0f,              
        .accelerationRejection = 10.0f,        
        .magneticRejection = 10.0f,            
        .rejectionTimeout = 5.0f,              
    };

    FusionAhrsSetSettings(&ahrs, &settings);



    xTaskCreatePinnedToCore(TaskGNSS, "GNSS", 4096, NULL, 1, NULL, 0);
    
    xTaskCreatePinnedToCore(TaskIMU, "IMU", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(TaskBMP, "Barometer", 4096, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(TaskFlightControl, "PID", 8192, NULL, 3, NULL, 1); 

    vTaskDelete(NULL);
}

void loop() {
  vTaskDelete(NULL); 
}
