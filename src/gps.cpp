#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "TinyGPSPlus.h" // GPS NMEA sentence parser.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "time_manager.h" // Header file for time management tasks.


static TinyGPSPlus gps; // Object that parses NMEA sentences from the NEO-6M GPS module
static bool time_was_set_by_gps = false; // Flag to check if the time was set by GPS


static void commandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (STRINGS_ARE_EQUAL(command, "gps")) {
        Serial.printf("\n[GPS]Reading GPS data\n");
        TinyGPSPlus* gps = (TinyGPSPlus*)handler_args;
        constexpr float invalid_value = -1.0f; // Begin the fields with arbitrated invalid value and update them if the gps data is valid.
        float latitude = invalid_value;
        float longitude = invalid_value;
        float speed = invalid_value;
        float course = invalid_value;
        uint8_t satellites = 0;

        if (gps->location.isValid()) {
            latitude = gps->location.lat();
            longitude = gps->location.lng();
            Serial.printf("[GPS]Latitude: %f, Longitude: %f\n", latitude, longitude);
        }
        if (gps->speed.isValid()) { 
            speed = gps->speed.kmph();
            Serial.printf("[GPS]Speed: %f\n", speed);
        }
        if (gps->course.isValid()) {
            course = gps->course.deg();
            Serial.printf("[GPS]Course: %f\n", course);
        }
        if (gps->satellites.isValid()) {
            satellites = gps->satellites.value();
            Serial.printf("[GPS]Satellites: %d\n", satellites);
        }

    }
}

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

    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, COMMAND_BASE, ESP_EVENT_ANY_ID, commandCallback, &gps); 

    while (true) {
        while (Serial2.available()) {
            // Reads the serial stream from the NEO-6M GPS module and parses it into TinyGPSPlus object if a valid NMEA sentence is received
            if (gps.encode(Serial2.read())) { 
                constexpr float invalid_value = -1.0f; // Begin the fields with arbitrated invalid value and update them if the gps data is valid.
                float latitude = invalid_value;
                float longitude = invalid_value;

                if (gps.location.isValid()) {
                    latitude = gps.location.lat();
                    longitude = gps.location.lng();
                    PrintPosition(latitude, longitude, SECONDS(10));
                }

                check_time_synchronization();
            }           
            vTaskDelay(pdMS_TO_TICKS(50)); // Allow other tasks to run
        }
    }
}