#pragma once
#include <Arduino.h>
#include "data_bms.h"
#include "data_motor.h"
#include "data_gps.h"
#include "data_temperatures.h"
#include "data_mppt.h"
#include "data_instrumentation.h"
#include "data_bms.h"
#include "data_propulsion.h"

// A dedicated struct for timestamping
typedef struct {
    uint64_t time_since_boot_ms;  // Milliseconds since boot (for local logging/timing)
    uint32_t epoch_seconds;       // Seconds since Unix epoch (for radio transmission)
    uint16_t epoch_ms;     // Millisecond part of the second (0-999)
} timestamp_data_t;

typedef enum {
    DATA_SOURCE_BMS,
    DATA_SOURCE_MOTOR_LEFT,
    DATA_SOURCE_MOTOR_RIGHT,
    DATA_SOURCE_GPS,
    DATA_SOURCE_MPPT,
    DATA_SOURCE_INSTRUMENTATION,
    DATA_SOURCE_PROPULSION,
    DATA_SOURCE_TEMPERATURES 
} data_source_t;

//This must be kept in sync and in the same order as the data_source_t enum
const char* const DATA_SOURCE_NAMES[] = {
    "BMS",
    "Motor Left",
    "Motor Right",
    "GPS",
    "MPPT",
    "Instrumentation",
    "Propulsion",
    "Temperatures"
};

typedef struct {
    data_source_t source;
    union {
        bms_data_t bms;
        motor_data_t motor;
        gps_data_t gps;
        mppt_data_t mppt;
        instrumentation_data_t instrumentation;
        propulsion_data_t propulsion;
        temperature_data_t temperature;
    } payload;
    
    timestamp_data_t timestamp; // Timestamp data for when the message was created
} message_t;


