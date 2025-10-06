#pragma once
#include <cstdint>
#include "data.hpp"
#include "arariboat/mavlink.h"

// Enum for each MAVLink message type that needs independent throttling.
// This is shared between the CAN and Radio transmitter tasks.
enum ThrottledMessage {
    MSG_BMS,
    MSG_BMS_STATUS,
    MSG_MOTOR_I_LEFT,
    MSG_MOTOR_II_LEFT,
    MSG_MOTOR_I_RIGHT,
    MSG_MOTOR_II_RIGHT,
    MSG_MPPT,
    MSG_MPPT_STATE,
    MSG_GPS,
    MSG_INSTRUMENTATION,
    MSG_TEMPERATURES,
    MSG_STRINGS_MPPT,
    NUM_THROTTLED_MESSAGES // Must be last, provides the size of the enum.
};

// Global array defining the minimum interval in milliseconds between sending
// messages for each corresponding type in the ThrottledMessage enum.
const uint32_t send_intervals_ms[] = {
    [MSG_BMS]               = 1000,  // 1 Hz
    [MSG_BMS_STATUS]        = 5000,  // 0.2 Hz
    [MSG_MOTOR_I_LEFT]      = 500,   // 2 Hz
    [MSG_MOTOR_II_LEFT]     = 2000,  // 0.5 Hz
    [MSG_MOTOR_I_RIGHT]     = 500,   // 2 Hz
    [MSG_MOTOR_II_RIGHT]    = 2000,  // 0.5 Hz
    [MSG_MPPT]              = 1000,  // 1 Hz
    [MSG_MPPT_STATE]        = 5000,  // 0.2 Hz
    [MSG_GPS]               = 1000,  // 1 Hz
    [MSG_INSTRUMENTATION]   = 1000,  // 1 Hz
    [MSG_TEMPERATURES]      = 10000, // 0.1 Hz
    [MSG_STRINGS_MPPT]       = 200,  // 5 Hz
};


// Dictionary to map each ThrottledMessage enum value to its corresponding data source.
const data_source_t throttled_message_sources[] = {
    [MSG_BMS] = DATA_SOURCE_BMS,
    [MSG_BMS_STATUS] = DATA_SOURCE_BMS,
    [MSG_MOTOR_I_LEFT] = DATA_SOURCE_MOTOR_LEFT,
    [MSG_MOTOR_II_LEFT] = DATA_SOURCE_MOTOR_LEFT,
    [MSG_MOTOR_I_RIGHT] = DATA_SOURCE_MOTOR_RIGHT,
    [MSG_MOTOR_II_RIGHT] = DATA_SOURCE_MOTOR_RIGHT,
    [MSG_MPPT] = DATA_SOURCE_MPPT,
    [MSG_MPPT_STATE] = DATA_SOURCE_MPPT,
    [MSG_GPS] = DATA_SOURCE_GPS,
    [MSG_INSTRUMENTATION] = DATA_SOURCE_INSTRUMENTATION,
    [MSG_TEMPERATURES] = DATA_SOURCE_TEMPERATURES,
    [MSG_STRINGS_MPPT] = DATA_SOURCE_MPPT_STRINGS,
}; 

// Function to get the data source for a given ThrottledMessage enum value.
static inline data_source_t get_data_source_for_message(ThrottledMessage message) {
    if (message < 0 || message >= NUM_THROTTLED_MESSAGES) {
        return DATA_SOURCE_BMS; // Default to BMS if out of range
    }
    return throttled_message_sources[message];
};
