#pragma once
#include <Arduino.h>

typedef struct {
    int16_t battery_current_cA;
    int16_t motor_current_left_cA;
    int16_t motor_current_right_cA;
    int16_t mppt_current_cA;
    uint16_t battery_voltage_cV;
    uint16_t auxiliary_battery_voltage_cV;
    uint16_t irradiance; // [W/m^2]
    uint32_t timestamp_ms;
} instrumentation_data_t;