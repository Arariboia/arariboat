#pragma once
#include <Arduino.h>

// Holds current data from solar panel strings
typedef struct {
    uint16_t string_1;
    uint16_t string_2;
    uint16_t string_3;
    uint16_t string_4;
} mppt_strings_data_t;

typedef struct {
    int16_t battery_current_cA;
    int16_t motor_current_left_cA;
    int16_t motor_current_right_cA;
    int16_t mppt_current_cA;
    mppt_strings_data_t panel_strings_mA;
    uint16_t battery_voltage_cV;
    uint16_t auxiliary_battery_voltage_cV;
    uint16_t auxiliary_battery_current_cA;
    uint16_t irradiance; // [W/m^2]
    uint32_t timestamp_ms;
} instrumentation_data_t;