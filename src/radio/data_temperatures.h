#pragma once
#include <Arduino.h>

//Temperatures are sent as centidegrees Celsius (cdegC)
typedef struct {
    int16_t battery_left_cdegC; // Left side of battery pack
    int16_t battery_right_cdegC; // Right side of battery pack
    int16_t mppt_left_cdegC; //  MPPT left side
    int16_t mppt_right_cdegC; // MPPT right side
} temperature_data_t;

