#include "readgnss.h"
#include <string.h>
#include <stdlib.h>

HardwareSerial GNSS(GNSS_SERIAL_PORT);

GNSSData masterGNSSData;
SatelliteData masterSatellites[MAX_SATELLITES];

enum NMEA_State { WAIT_DOLLAR, READ_HEADER, READ_PAYLOAD };

bool readNMEA(HardwareSerial* GNSS) {
    static NMEA_State currentState = WAIT_DOLLAR;
    static char buffer[30]; 
    static int charIndex = 0;
    static int fieldIndex = 0;
    static int sentenceType = 0; // 1=GPRMC, 2=GPGGA, 3=GPGSV
    
    static GNSSData tempGNSSData; 
    static SatelliteData tempSatellites[MAX_SATELLITES];

    static int gsvTotalMessages = 1;
    static int gsvMessageNum = 1;
    
    bool newFixAvailable = false;

    while (GNSS->available() > 0) {
        char c = GNSS->read();

        switch (currentState) {
            
            case WAIT_DOLLAR:
                if (c == '$') {
                    charIndex = 0;
                    sentenceType = 0;
                    currentState = READ_HEADER;
                }
                break;

            case READ_HEADER:
                if (c == ',') {
                    buffer[charIndex] = '\0';
                    fieldIndex = 1; // First data field is index 1
                    
                    if (strcmp(buffer, "GPRMC") == 0) sentenceType = 1;
                    else if (strcmp(buffer, "GPGGA") == 0) sentenceType = 2;
                    else if (strcmp(buffer, "GPGSV") == 0) sentenceType = 3;
                    else currentState = WAIT_DOLLAR; // Ignore other sentences
                    
                    if (sentenceType != 0) {
                        charIndex = 0;
                        currentState = READ_PAYLOAD;
                    }
                } else if (charIndex < 6) {
                    buffer[charIndex++] = c;
                } else {
                    currentState = WAIT_DOLLAR; 
                }
                break;

            case READ_PAYLOAD:
                if (c == ',' || c == '*') {
                    buffer[charIndex] = '\0'; 
                    
                    if (sentenceType == 1 && charIndex > 0) { 
                        if (fieldIndex == 1 && charIndex >= 6) { // Time
                            char temp[3] = {0};
                            strncpy(temp, &buffer[0], 2); tempGNSSData.time_Hours = atoi(temp);
                            strncpy(temp, &buffer[2], 2); tempGNSSData.time_Minutes = atoi(temp);
                            strncpy(temp, &buffer[4], 2); tempGNSSData.time_Seconds = atoi(temp);
                        }
                        else if (fieldIndex == 3 && charIndex >= 9) { // Latitude
                            char temp[6] = {0};
                            strncpy(temp, &buffer[0], 2); tempGNSSData.latitude_Degrees = atoi(temp);
                            strncpy(temp, &buffer[2], 2); tempGNSSData.latitude_Minutes = atoi(temp);
                            // Skip the decimal at buffer[4] and grab the next 5 digits
                            strncpy(temp, &buffer[5], 5); tempGNSSData.latitude_Seconds = atof(temp) * 0.0006f;
                        }
                        else if (fieldIndex == 4) { // Lat Direction
                            tempGNSSData.latitude_Direction = buffer[0];
                        }
                        else if (fieldIndex == 5 && charIndex >= 10) { // Longitude
                            char temp[6] = {0};
                            strncpy(temp, &buffer[0], 3); tempGNSSData.longitude_Degrees = atoi(temp);
                            strncpy(temp, &buffer[3], 2); tempGNSSData.longitude_Minutes = atoi(temp);
                            // Skip the decimal at buffer[5] and grab the next 5 digits
                            strncpy(temp, &buffer[6], 5); tempGNSSData.longitude_Seconds = atof(temp) * 0.0006f;
                        }
                        else if (fieldIndex == 6) { // Lon Direction
                            tempGNSSData.longitude_Direction = buffer[0];
                        }
                        else if (fieldIndex == 9 && charIndex >= 6) { // Date
                            char temp[3] = {0};
                            strncpy(temp, &buffer[0], 2); tempGNSSData.time_Day = atoi(temp);
                            strncpy(temp, &buffer[2], 2); tempGNSSData.time_Month = atoi(temp);
                            strncpy(temp, &buffer[4], 2); tempGNSSData.time_Year = atoi(temp);
                        }
                    }
                    
                    else if (sentenceType == 2 && charIndex > 0) { 
                        if (fieldIndex == 6) { // Fix Quality
                            tempGNSSData.fixQuality = atoi(buffer);
                        }
                        else if (fieldIndex == 7) { // Active Sats
                            tempGNSSData.satellite_number_active = atoi(buffer);
                        }
                        else if (fieldIndex == 8) { // HDOP
                            tempGNSSData.HDOP = atof(buffer);
                        }
                        else if (fieldIndex == 9) { // Altitude (Standardized to atof)
                            tempGNSSData.true_Altitude = atof(buffer);
                        }
                    }

                    else if (sentenceType == 3) {
                        if (fieldIndex == 1 && charIndex > 0) {
                            gsvTotalMessages = atoi(buffer);
                        }
                        else if (fieldIndex == 2 && charIndex > 0) {
                            gsvMessageNum = atoi(buffer);

                            if (gsvMessageNum == 1) {
                                memset(tempSatellites, 0, sizeof(tempSatellites));
                            }
                        }
                        else if (fieldIndex == 3 && charIndex > 0) {
                            tempGNSSData.satellite_number_visible = atoi(buffer);
                        }
                        else if (fieldIndex >= 4) {
                            int satBlock = (fieldIndex - 4) / 4; // 0, 1, 2, or 3 (up to 4 sats per message)
                            int property = (fieldIndex - 4) % 4; // 0=ID, 1=Elev, 2=Azim, 3=SNR
                            
                            int targetIndex = ((gsvMessageNum - 1) * 4) + satBlock;
                            
                            if (targetIndex < MAX_SATELLITES && charIndex > 0) {
                                if (property == 0) tempSatellites[targetIndex].satelliteID = atoi(buffer);
                                else if (property == 1) tempSatellites[targetIndex].elevation = atoi(buffer);
                                else if (property == 2) tempSatellites[targetIndex].azimuth = atoi(buffer);
                                else if (property == 3) tempSatellites[targetIndex].SNR = atoi(buffer);
                            }
                        }
                    }

                    charIndex = 0; 
                    fieldIndex++;

                    if (c == '*') { 
                        currentState = WAIT_DOLLAR;

                        if (sentenceType == 2) {
                            if (xSemaphoreTake(gnssMutex, 0) == pdTRUE) {
                                masterGNSSData = tempGNSSData;
                                xSemaphoreGive(gnssMutex);
                                newFixAvailable = true; 
                            }
                        }

                        if (sentenceType == 3 && gsvMessageNum == gsvTotalMessages) {
                            if (xSemaphoreTake(gnssMutex, 0) == pdTRUE) {
                                memcpy(masterSatellites, tempSatellites, sizeof(tempSatellites));
                                xSemaphoreGive(gnssMutex);
                            }
                        }
                    }
                } else if (charIndex < 29) {
                    buffer[charIndex++] = c;
                }
                break;
        }
    }
    return newFixAvailable;
}

void printGNSS(){
    if (xSemaphoreTake(gnssMutex, 10 / portTICK_PERIOD_MS) == pdTRUE) {
        
        Serial.println("====== GNSS TELEMETRY ======");
        
        // 2. Print Time and Date using %02d to force leading zeros
        Serial.printf("Time: %02d:%02d:%02d | Date: %02d/%02d/%02d\n", 
            masterGNSSData.time_Hours, masterGNSSData.time_Minutes, masterGNSSData.time_Seconds,
            masterGNSSData.time_Day, masterGNSSData.time_Month, masterGNSSData.time_Year);
            
        // 3. Print Coordinates using %d for bytes/ints, %f for floats, %c for chars
        Serial.printf("Lat: %d deg %d min %.4f sec %c\n",
            masterGNSSData.latitude_Degrees, masterGNSSData.latitude_Minutes, 
            masterGNSSData.latitude_Seconds, masterGNSSData.latitude_Direction);
            
        Serial.printf("Lon: %d deg %d min %.4f sec %c\n",
            masterGNSSData.longitude_Degrees, masterGNSSData.longitude_Minutes, 
            masterGNSSData.longitude_Seconds, masterGNSSData.longitude_Direction);
            
        // 4. Print Fix and Altitude
        Serial.printf("Fix: %d | Sats: %d | HDOP: %.2f | Alt: %.1fm\n",
            masterGNSSData.fixQuality, masterGNSSData.satellite_number_visible, 
            masterGNSSData.HDOP, masterGNSSData.true_Altitude);
            
        // 5. Loop through the array of structs for the satellites
        int active = masterGNSSData.satellite_number_visible;
        if (active > MAX_SATELLITES) active = MAX_SATELLITES;
        
        Serial.println("--- Visible Satellites ---");
        for (int i = 0; i < active; i++) {
            // Only print if the ID is valid
            if (masterSatellites[i].satelliteID != 0) { 
                Serial.printf("ID: %02d | Elev: %02d | Azim: %03d | SNR: %02d\n",
                    masterSatellites[i].satelliteID, masterSatellites[i].elevation, 
                    masterSatellites[i].azimuth, masterSatellites[i].SNR);
            }
        }
        Serial.println("==========================\n");
        
        // 6. Give the key back!
        xSemaphoreGive(gnssMutex);
    }
}