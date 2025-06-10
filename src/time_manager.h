#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include "ESP32Time.h" // Include the ESP32Time library for RTC functionality

// The RTC object is declared here for use across the project.
// It is defined in the main file, since other tasks also use it, and provides access to the ESP32's real-time clock.
extern ESP32Time RTC;

// --- System-Wide Event Group ---
// This handle will be created in the main file and shared with tasks.
extern EventGroupHandle_t system_event_group;

// --- Event Bits Definition ---
// A bit to signal that the WiFi has successfully connected.
const EventBits_t BIT_WIFI_CONNECTED = (1 << 0);
// A bit to signal that the system time has been synchronized from ANY valid source (NTP or GPS).
const EventBits_t BIT_TIME_SYNC_SUCCESS = (1 << 1);

// This is our "Time Provider" function. It's the single source of truth for the time.
// It is fully decoupled from HOW the time is obtained.
inline uint32_t getSystemTimestamp() {
    // Check the event group to see if the time has been synchronized.
    EventBits_t bits = xEventGroupGetBits(system_event_group);
    if ((bits & BIT_TIME_SYNC_SUCCESS) != 0) {
        return RTC.getEpoch();
    }
    return millis(); // Return time in milliseconds since boot if time is not synchronized.
}


// --- Task Declaration ---

/**
 * @brief Manages acquiring the system time with a primary (NTP) and fallback (GPS) source.
 * * This task attempts to get time from an NTP server first. If that fails or if
 * WiFi is unavailable, it will wait for another task (like the GPSTask) to 
 * signal that time has been acquired via the system_event_group.
 * * @param parameter Task parameters (unused).
 */
void TimeManagerTask(void* parameter);

