#pragma once
#include <Arduino.h>

//Temperatures are sent as centidegrees Celsius (cdegC)
typedef struct {
    int16_t battery_left_cdegC; // Left side of battery pack
    int16_t battery_right_cdegC; // Right side of battery pack
    int16_t mppt_left_cdegC; //  MPPT left side
    int16_t mppt_right_cdegC; // MPPT right side
    int16_t motor_left_cdegC; // Motor left side
    int16_t motor_right_cdegC; // Motor right side
    int16_t esc_left_cdegC; // ESC left side
    int16_t esc_right_cdegC; // ESC right side
    int16_t motor_cover_left_cdegC; // Motor cover left side
    int16_t motor_cover_right_cdegC; // Motor cover right side
} temperature_data_t;

