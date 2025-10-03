#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "arariboat/mavlink.h"

// Struct to hold all relevant radio statistics.
typedef struct {
    int rssi;
    uint16_t packet_rx_success_count;
    uint16_t packet_rx_drop_count;
    uint8_t buffer_overrun;
    uint8_t parse_error;
    uint8_t last_msg_id;
    char last_msg_name[15]; // To store the name of the last message
} radio_stats_t;

class RadioManager {
public:
    // Singleton access method
    static RadioManager& GetInstance() {
        static RadioManager instance;
        return instance;
    }

    // Deleted constructors and assignment operators to prevent copies
    RadioManager(RadioManager const&) = delete;
    void operator=(RadioManager const&) = delete;

    // Method to update the statistics from the radio task
    void update_statistics(const mavlink_status_t& status, int rssi, int msg_id) {
        // Lock the mutex to ensure thread-safe access
        if (xSemaphoreTake(stats_mutex, portMAX_DELAY) == pdTRUE) {
            stats.rssi = rssi;
            stats.packet_rx_success_count = status.packet_rx_success_count;
            stats.packet_rx_drop_count = status.packet_rx_drop_count;
            stats.buffer_overrun = status.buffer_overrun;
            stats.parse_error = status.parse_error;
            stats.last_msg_id = msg_id;

            // A simple way to get message name from ID
            switch(msg_id) {
                case MAVLINK_MSG_ID_BMS:
                    strcpy(stats.last_msg_name, "BMS");
                    break;
                case MAVLINK_MSG_ID_BMS_STATUS:
                     strcpy(stats.last_msg_name, "BMS_STATUS");
                    break;
                case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I:
                     strcpy(stats.last_msg_name, "MOTOR_I");
                    break;
                case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II:
                     strcpy(stats.last_msg_name, "MOTOR_II");
                    break;
                default:
                     snprintf(stats.last_msg_name, sizeof(stats.last_msg_name), "ID %d", msg_id);
                    break;
            }

            // Release the mutex
            xSemaphoreGive(stats_mutex);
        }
    }

    // Getter method to retrieve all statistics at once
    radio_stats_t get_statistics() {
        radio_stats_t current_stats;
        // Lock the mutex for a thread-safe read
        if (xSemaphoreTake(stats_mutex, portMAX_DELAY) == pdTRUE) {
            current_stats = stats;
            // Release the mutex
            xSemaphoreGive(stats_mutex);
        }
        return current_stats;
    }

private:
    // Private constructor for the singleton pattern
    RadioManager() {
        // Create the mutex
        stats_mutex = xSemaphoreCreateMutex();
        // Initialize stats to a default state
        memset(&stats, 0, sizeof(radio_stats_t));
        strcpy(stats.last_msg_name, "N/A");
    }

    // Private destructor
    ~RadioManager() {
        if(stats_mutex != NULL) {
            vSemaphoreDelete(stats_mutex);
        }
    }

    radio_stats_t stats;
    SemaphoreHandle_t stats_mutex;
};
