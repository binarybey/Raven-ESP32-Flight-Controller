#include <barometer.h>
#include <math.h>

/*
dig_T1: 28177
dig_T2: 26653
dig_T3: -1000
dig_P1: 36838
dig_P2: -10478
dig_P3: 3024
dig_P4: 4373
dig_P5: 147
dig_P6: -7
dig_P7: 15500
dig_P8: -14600
dig_P9: 6000
*/

// The BMP180 got cancelled, the respective code for BMP180 may be obsolete or faulty.
const double P0 = 101500.0;


calibData280 BMP280calib;

bmpReadingsInt32 bmp_raw;

bmpReadingsDouble bmp_out;

void readCalibrationData280(calibData280 &calib)
{
    I2C_IO::read_IIC(BMP280_ADDRESS, 0x88, 12, (int16_t*)&calib, false);
}

void printCalibration280(const calibData280& calib) {
    Serial.printf("dig_T1: %u dig_T2: %d dig_T3: %d dig_P1: %u dig_P2: %d dig_P3: %d dig_P4: %d dig_P5: %d dig_P6: %d dig_P7: %d dig_P8: %d dig_P9: %d\n",
            calib.dig_T1, calib.dig_T2, calib.dig_T3, calib.dig_P1, calib.dig_P2, calib.dig_P3, calib.dig_P4, calib.dig_P5, calib.dig_P6, calib.dig_P7, calib.dig_P8, calib.dig_P9);
}

void initBMP280() {
    I2C_IO::write_IIC(0x76, 0xF5, 0x10);
    I2C_IO::write_IIC(0x76, 0xF4, 0x57);
}

bool readBMP280(const calibData280 &calib, bmpReadingsInt32 &raw_readings, bmpReadingsDouble &output) {
    if (I2C_IO::read_IIC_20bit(BMP280_ADDRESS, BMP280_OUTPUT_REG, 2, (int32_t*)&raw_readings)) {
        int32_t adc_p = raw_readings.pressure;
        int32_t adc_t = raw_readings.temperature;

        double var1, var2, t_fine;

        var1 = (((double)adc_t) / 16384.0 - ((double)calib.dig_T1) / 1024.0) * ((double)calib.dig_T2);
        var2 = ((((double)adc_t) / 131072.0 - ((double)calib.dig_T1) / 8192.0) * 
       (((double)adc_t) / 131072.0 - ((double)calib.dig_T1) / 8192.0)) * ((double)calib.dig_T3);

        t_fine = var1 + var2;
        output.temperature = (var1 + var2) / 5120.0; 
        var1 = (t_fine / 2.0) - 64000.0;
        var2 = var1 * var1 * ((double)calib.dig_P6) / 32768.0;
        var2 = var2 + var1 * ((double)calib.dig_P5) * 2.0;
        var2 = (var2 / 4.0) + (((double)calib.dig_P4) * 65536.0);
        var1 = (((double)calib.dig_P3) * var1 * var1 / 524288.0 + ((double)calib.dig_P2) * var1) / 524288.0;
        var1 = (1.0 + var1 / 32768.0) * ((double)calib.dig_P1);

        if (var1 == 0.0) {
            return false;
        }

        output.pressure = 1048576.0 - (double)adc_p;
        output.pressure = (output.pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)calib.dig_P9) * output.pressure * output.pressure / 2147483648.0;
        var2 = output.pressure * ((double)calib.dig_P8) / 32768.0;

        output.pressure = output.pressure + (var1 + var2 + ((double)calib.dig_P7)) / 16.0;
        output.altitude = 44330.0 * (1.0 - pow(output.pressure / P0, 0.1903));

        return true;
    }
    return false;
}

void printPressTemp(const bmpReadingsDouble &outvals) {
    Serial.printf("Pressure:    %f      Temperature:   %f       Elevation:    %f\n", outvals.pressure, outvals.temperature, outvals.altitude);
}

void baroToRaw(const BaroSnapshot &baro, int64_t now_us, raven::RawSensors &raw) {
    raw.baro_valid         = baro.valid && (now_us - baro.sample_us) < BARO_FRESH_US;
    raw.baro_altitude      = baro.altitude;
    raw.baro_climb_rate    = baro.climb_rate;
    raw.baro_pressure_pa   = baro.pressure_pa;
    raw.baro_temperature_c = baro.temperature_c;
}