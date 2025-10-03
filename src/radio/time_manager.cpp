#include "time_manager.h"
#include <ESP32Time.h>

// This RTC object is declared as extern in the main .ino file.
// It's the single source of truth for the current time.
extern ESP32Time RTC;

void time_manager_task(void* parameter) {
    Serial.println("[TimeManager] Task started.");

    // This task will now loop until time is synchronized from any source.
    while (true) {
        // First, check if time has already been synced by another task (like GPS).
        EventBits_t bits = xEventGroupGetBits(system_event_group);
        if ((bits & BIT_TIME_SYNC_SUCCESS) != 0) {
            Serial.println("[TimeManager] Time was synchronized by another source. Task is complete.");
            break; // Exit the loop
        }

        Serial.println("[TimeManager] Waiting for WiFi connection (45s timeout)...");
        // Wait for the WiFi to connect.
        bits = xEventGroupWaitBits(
            system_event_group,       // The event group being tested.
            BIT_WIFI_CONNECTED,       // The bits to wait for.
            pdFALSE,                  // Don't clear bits on exit.
            pdTRUE,                   // Wait for ALL bits to be set.
            pdMS_TO_TICKS(45000)      // Maximum wait time for this cycle.
        );

        // Check if the WiFi connected bit was set during our wait
        if ((bits & BIT_WIFI_CONNECTED) != 0) {
            Serial.println("[TimeManager] WiFi connected. Attempting NTP sync...");
            

            constexpr long gmt_seconds_offset = 0;
            constexpr int daylight_seconds_offset = 0;
            configTime(gmt_seconds_offset, daylight_seconds_offset, "south-america.pool.ntp.org");
            struct tm timeinfo;

            // Attempt to get the time from the NTP server
            if (getLocalTime(&timeinfo, 10000)) { // 10-second timeout
                RTC.setTimeStruct(timeinfo);
                // Signal to the rest of the system that time is now synchronized.
                xEventGroupSetBits(system_event_group, BIT_TIME_SYNC_SUCCESS);
                Serial.printf("[TimeManager] NTP sync successful: %s\n", RTC.getDateTime(true).c_str());
                break; // Exit the loop, our job is done.
            } else {
                Serial.println("[TimeManager] NTP sync failed. Will retry after a delay.");
            }
        } else {
            Serial.println("[TimeManager] Timed out waiting for WiFi. Will retry after a delay.");
        }
        
        // If we've reached here, NTP sync failed or WiFi wasn't available.
        // Wait for 60 seconds before cycling again to give GPS a chance and avoid spamming.
        Serial.println("[TimeManager] Pausing before retry.");
        vTaskDelay(pdMS_TO_TICKS(30000));
    }

    Serial.println("[TimeManager] Task's job is done. Deleting self.");
    vTaskDelete(NULL); // The task is no longer needed.
}
