#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "time_manager.h" // Header file for time management tasks.
#include "data_gps.h" // Header file for GPS data structure.
#include "data.hpp" // Header file for data structures.
#include "queues.hpp" // Header file for queue initialization and definitions.


static TinyGPSPlus gps; // Object that parses NMEA sentences from the NEO-6M GPS module
static bool time_was_set_by_gps = false; // Flag to check if the time was set by GPS

const int queue_post_interval_ms = 500;

#define SECONDS(x) (x*1000)
static void PrintPosition(float latitude, float longitude, int interval) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < interval) return;
    lastPrint = millis();

    DEBUG_PRINTF("\n[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
}

// Function to check if the GPS time is valid and set the system time accordingly
static void check_time_synchronization() {

    if (time_was_set_by_gps) {
        // If the time was already set by GPS, no need to check again
        return;
    }

    if (gps.date.isValid() && gps.time.isValid()) {
        //Check if another source (like NTP) has already set the time
        EventBits_t current_bits = xEventGroupGetBits(system_event_group);  
        if ((current_bits & BIT_TIME_SYNC_SUCCESS) == 0) {
            Serial.printf("[GPS]Setting system time from GPS: %02d/%02d/%04d %02d:%02d:%02d\n", 
            gps.date.day(), gps.date.month(), gps.date.year(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
        
            tm timeinfo;
            timeinfo.tm_year = gps.date.year() - 1900; // tm_year is years since 1900
            timeinfo.tm_mon = gps.date.month() - 1; // tm_mon is 0-11
            timeinfo.tm_mday = gps.date.day();
            timeinfo.tm_hour = gps.time.hour();
            timeinfo.tm_min = gps.time.minute();
            timeinfo.tm_sec = gps.time.second();

            RTC.setTimeStruct(timeinfo); // Set the time in the RTC
            time_was_set_by_gps = true; // Set the flag to true to avoid setting the time again

            // Set the event bit to indicate time sync success
            xEventGroupSetBits(system_event_group, BIT_TIME_SYNC_SUCCESS);
            Serial.printf("[GPS]System time set successfully.\n");    
        }             
    }
}

void GPSTask(void* parameter) {

    // Example of latitude: 40.741895 (north is positive)
    // Example of longitude: -73.989308 (west is negative)
    // The fifth decimal place is worth up to 1.1 m. The sixth decimal place is worth up to 11cm. And so forth.
    
    constexpr uint8_t pin_gps_rx = PIN_GPS_RX;  
    constexpr uint8_t pin_gps_tx = PIN_GPS_TX; 
    constexpr int32_t baud_rate = 9600; // Fixed baud rate used by NEO-6M GPS module
    Serial2.begin(baud_rate, SERIAL_8N1, pin_gps_rx, pin_gps_tx);

    while (true) {

        if (!Serial2.available()) {
            // If no data is available, we can skip the rest of the loop
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        while (Serial2.available()) {
            if (gps.encode(Serial2.read())) {
                break; // Break if a complete sentence is parsed
            }
        }

        check_time_synchronization();

        gps_data_t gps_data = {0};
        char buffer[256];
        int len = 0;

        if (gps.location.isValid()) {
            gps_data.latitude_degE7 = static_cast<int32_t>(gps.location.lat() * 1e7);
            gps_data.longitude_degE7 = static_cast<int32_t>(gps.location.lng() * 1e7);
            len += snprintf(buffer + len, sizeof(buffer) - len,
                "[GPS]Latitude: %d, Longitude: %d\n", gps_data.latitude_degE7, gps_data.longitude_degE7);
        }

        if (gps.speed.isValid()) {
            gps_data.speed_cm_s = static_cast<uint16_t>(gps.speed.kmph() * (100000.0f / 3600.0f));
            len += snprintf(buffer + len, sizeof(buffer) - len,
                "[GPS]Speed: %d cm/s\n", gps_data.speed_cm_s);
        }

        if (gps.course.isValid()) {
            gps_data.course = static_cast<uint8_t>(gps.course.deg());
            len += snprintf(buffer + len, sizeof(buffer) - len,
                "[GPS]Course: %d\n", gps_data.course);
        }

        if (gps.satellites.isValid()) {
            gps_data.satellites_visible = static_cast<uint8_t>(gps.satellites.value());
            len += snprintf(buffer + len, sizeof(buffer) - len,
                "[GPS]Satellites: %d\n", gps_data.satellites_visible);
        }

        if (gps.hdop.isValid()) {
            gps_data.hdop_deciunits = static_cast<uint8_t>(gps.hdop.hdop() * 10.0f);
            len += snprintf(buffer + len, sizeof(buffer) - len,
                "[GPS]HDOP: %d\n", gps_data.hdop_deciunits);
        }

        Serial.print(buffer);

        static unsigned long last_post_time = 0;
        if (millis() - last_post_time < queue_post_interval_ms) {
            continue;
        }

        message_t msg;
        msg.source = DATA_SOURCE_GPS;
        msg.payload.gps = gps_data;
        msg.timestamp.time_since_boot_ms = millis();
        msg.timestamp.epoch_seconds = get_epoch_seconds();
        msg.timestamp.epoch_ms = get_epoch_millis();

        if (xQueueSend(broker_queue, &msg, pdMS_TO_TICKS(20)) != pdTRUE) {
            Serial.println("[GPS]Error: Queue is full)");
        }
    }
}