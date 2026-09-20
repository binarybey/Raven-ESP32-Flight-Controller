#pragma once
#include <Wire.h>
#include <Arduino.h>

class I2C_IO {
public:
    // Both functions inside the class. Passed by value. 
    // Added bigEndian parameter with a default value.
    static void write_IIC(uint8_t IICaddress, uint8_t IICregister, uint8_t IICkeyvalue);
    
    // Instead of returning a vector, we pass an existing buffer by reference to avoid heap fragmentation
    static bool read_IIC(uint8_t IICaddress, uint8_t IICregister, size_t count, int16_t* buffer, bool bigEndian = true);

    static bool read_IIC_20bit(uint8_t IICaddress, uint8_t IICregister, size_t count, int32_t* buffer);
};

void scanI2CBus();