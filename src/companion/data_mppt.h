#pragma once
#include <Arduino.h>

// --- Data Structures for MPPT Data ---

// Holds the core electrical readings as raw integer values.
// These values are scaled by 100 (e.g., cV = centi-volts).
typedef struct {
    uint16_t pv_voltage_cV;
    int16_t  pv_current_cA;
    uint16_t battery_voltage_cV;
    int16_t  battery_current_cA;
} mppt_electrical_data_t;

// Holds the status flag registers.
typedef struct {
    uint16_t battery_status;
    uint16_t charging_equipment_status;
} mppt_state_data_t;

// The main data structure that combines all data with a timestamp.
// This is the object that will be passed through the system (e.g., in queues).
typedef struct {
    mppt_electrical_data_t electrical;
    mppt_state_data_t      state;
    uint32_t               timestamp_ms;
} mppt_data_t;