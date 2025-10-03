#pragma once
#include <Arduino.h>

typedef struct {
    int32_t latitude_degE7; // Latitude in degrees * 10^7
    int32_t longitude_degE7; // Longitude in degrees * 10^7
    uint16_t speed_cm_s; // Speed in cm/s
    uint8_t course; //Direction of movement in degrees (0-360)
    uint8_t heading; //Direction of the bow in degrees (0-360)
    uint8_t satellites_visible; // Number of satellites visible
    uint8_t hdop_deciunits; // Indicates quality of GPS constellation, values below 5 are good, above 10 are poor. Values are scaled by 10 (e.g 5.0 = 50, 1.0 = 10, etc.)
} gps_data_t;