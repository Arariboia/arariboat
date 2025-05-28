#pragma once
#include <Arduino.h>

struct mppt_electrical_t {
    uint16_t pv_voltage_cV;
    int16_t pv_current_cA;
    uint16_t battery_voltage_cV;
    int16_t battery_current_cA;
};

struct mppt_state_t {
    uint16_t battery_status;
    uint16_t charging_equipment_status;
};

struct mppt_data_t {
    mppt_electrical_t electrical;
    mppt_state_t state;
    uint32_t timestamp_ms;
};