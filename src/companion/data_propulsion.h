#pragma once

// Propulsion data structure
typedef struct {
    float backup_potentiometer_volts; // Volts from backup potentiometer
    float helm_potentiometer_volts; // Volts from helm potentiometer
    float throttle_left_potentiometer_volts; // Volts from left throttle potentiometer
    float throttle_right_potentiometer_volts; // Volts from right throttle potentiometer
} propulsion_data_t;