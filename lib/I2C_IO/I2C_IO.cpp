#include "I2C_IO.h"

void I2C_IO::write_IIC(uint8_t IICaddress, uint8_t IICregister, uint8_t IICkeyvalue){
    Wire.beginTransmission(IICaddress);
    Wire.write(IICregister);
    Wire.write(IICkeyvalue);
    Wire.endTransmission();
}

// Returns a boolean indicating success, fills the provided buffer
bool I2C_IO::read_IIC(uint8_t IICaddress, uint8_t IICregister, size_t count, int16_t* buffer, bool bigEndian) {
    Wire.beginTransmission(IICaddress);
    Wire.write(IICregister);
    if (Wire.endTransmission(false) != 0) {
        return false; // Device didn't acknowledge
    }

    // Request the bytes (count * 2)
    size_t requestedBytes = count * 2;
    size_t receivedBytes = Wire.requestFrom(IICaddress, requestedBytes);

    if (receivedBytes < requestedBytes) {
        return false; // Did not receive the expected amount of data
    }

    for (size_t i = 0; i < count && Wire.available() >= 2; i++) {
        if (bigEndian) {
            uint8_t MSB = Wire.read();
            uint8_t LSB = Wire.read();
            buffer[i] = static_cast<int16_t>((MSB << 8) | LSB);
        } else {
            uint8_t LSB = Wire.read();
            uint8_t MSB = Wire.read();
            buffer[i] = static_cast<int16_t>((MSB << 8) | LSB);
        }
    }

    return true; // Success
}

bool I2C_IO::read_IIC_20bit(uint8_t IICaddress, uint8_t IICregister, size_t count, int32_t* buffer) {
    Wire.beginTransmission(IICaddress);
    Wire.write(IICregister);
    if (Wire.endTransmission(false) != 0) {
        return false; // Device didn't acknowledge
    }

    // Request the bytes (count * 3, because each 20-bit reading takes 3 registers)
    size_t requestedBytes = count * 3;
    size_t receivedBytes = Wire.requestFrom(IICaddress, requestedBytes);

    if (receivedBytes != requestedBytes) {
        return false; // Did not receive the expected amount of data
    }

    for (size_t i = 0; i < count && Wire.available() >= 3; i++) {
        uint8_t MSB = Wire.read();
        uint8_t LSB = Wire.read();
        uint8_t XLSB = Wire.read();

        // Cast to int32_t BEFORE shifting to prevent 16-bit integer overflow
        buffer[i] = ((int32_t)MSB << 12) | ((int32_t)LSB << 4) | (XLSB >> 4);
    }

    return true; // Success
}

void scanI2CBus() {
    byte error, address;
    int nDevices = 0;

    Serial.println("\n--- I2C Bus Scanner ---");
    Serial.println("Scanning...");

    // I2C addresses range from 1 to 127
    for(address = 1; address < 127; address++) {
        
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("Device found at address 0x");
            if (address < 16) {
                Serial.print("0"); // pad single-digit hex with a zero
            }
            Serial.println(address, HEX);
            nDevices++;
        }
        else if (error == 4) {
            Serial.print("Unknown error at address 0x");
            if (address < 16) {
                Serial.print("0");
            }
            Serial.println(address, HEX);
        }
    }
    
    if (nDevices == 0) {
        Serial.println("No I2C devices found.\n");
    } else {
        Serial.println("Scan complete.\n");
    }
}