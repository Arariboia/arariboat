#pragma once
#include <Arduino.h>
#include "data_bms.h"
#include "data_motor.h"
#include "data_gps.h"
#include "data_temperatures.h"
#include "data_mppt.h"

typedef enum {
    DATA_SOURCE_BMS,
    DATA_SOURCE_MOTOR_LEFT,
    DATA_SOURCE_MOTOR_RIGHT,
    DATA_SOURCE_GPS,
    DATA_SOURCE_MPPT,
    DATA_SOURCE_INSTRUMENTATION,
    DATA_SOURCE_TEMPERATURES 
} data_source_t;
