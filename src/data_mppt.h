#pragma once
#include <Arduino.h>

typedef struct {
    uint16_t pv_voltage_cV;
    int16_t pv_current_cA;
    uint16_t battery_voltage_cV;
    int16_t battery_current_cA;
} mppt_electrical_data_t;

typedef struct {
    uint16_t battery_status;
    uint16_t charging_equipment_status;
} mppt_state_data_t;

struct mppt_data_t {
    mppt_electrical_data electrical;
    mppt_state_data state;
    uint32_t timestamp_ms;
};