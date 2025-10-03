#include <Arduino.h>
#include <Wire.h> 
#include "SSD1306Wire.h"
#include "radio_manager.h" 
// #include "wifi_manager.hpp" // Assuming WiFiManager exists elsewhere

// Enum to manage the different pages on the display.
enum Page {
    Home,
    Lora,
    // Wifi, // Assuming a WiFi page might be added back later
    NumberPages
};

// --- Screen Drawing Functions ---

static void ShowHomeScreen(SSD1306Wire& screen) {
    constexpr uint8_t vertical_offset = 16;
    screen.clear();
    screen.setFont(ArialMT_Plain_16);
    screen.setTextAlignment(TEXT_ALIGN_CENTER);
    screen.drawString(screen.getWidth() / 2, (screen.getHeight() - vertical_offset) / 2, "LoRa Receiver");
    screen.display();
}

static void ShowLoraScreen(SSD1306Wire& screen) {
    screen.clear();
    screen.setFont(ArialMT_Plain_10);
    screen.setTextAlignment(TEXT_ALIGN_LEFT);
    screen.drawString((screen.getWidth() - 40) / 2, 0, "[LoRa Stats]");

    // Get the latest stats from the RadioManager
    radio_stats_t stats = RadioManager::GetInstance().get_statistics();

    // Display the stats
    screen.drawString(0, 12, "Last Msg: " + String(stats.last_msg_name));
    screen.drawString(0, 24, "RSSI: " + String(stats.rssi) + " dBm");
    screen.drawString(0, 36, "Rcvd: " + String(stats.packet_rx_success_count));
    screen.drawString(0, 48, "Drop: " + String(stats.packet_rx_drop_count) + " Err: " + String(stats.parse_error));

    screen.display();
}

// --- Page Control Logic ---

static int SetPageInterval(Page page) {
    int interval_ms;
    switch (page) {
        case Home:
            interval_ms = 2000; // Show home screen for 5 seconds
            break;
        case Lora:
            interval_ms = 30000; // Show LoRa stats for 5 seconds
            break;
        default:
            interval_ms = 2000;
            break;
    }
    return interval_ms;
}

static void SetPage(SSD1306Wire& screen, Page page) {
    switch (page) {
        case Home:
            static bool home_screen_initialized = false;
            if (!home_screen_initialized) {
                ShowHomeScreen(screen);
                home_screen_initialized = true;
            }
            break;
        case Lora:
            ShowLoraScreen(screen);
            break;
        default:
            ShowHomeScreen(screen); // Default to home screen
            break;
    }
}

/// @brief Task that controls the 128x64 OLED display via I2C.
/// It cycles through different informational pages.
/// @param parameter Not used.
void display_screen_task(void* parameter) {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Initial delay to allow other tasks to start
    Serial.printf("[DISPLAY] Starting display task...\n");

    // I2C pins and address for the TTGO LoRa v2.1.6
    constexpr uint8_t address = 0x3C;
    constexpr uint8_t sda_pin = GPIO_NUM_21;
    constexpr uint8_t scl_pin = GPIO_NUM_22;
    SSD1306Wire screen(address, sda_pin, scl_pin);

    screen.init();
    screen.flipScreenVertically();
    
    const uint16_t screen_update_rate_ms = 500;
    uint32_t last_page_change_time = 0;
    int current_page_index = 0;
    
    while (true) {
        Page current_page = static_cast<Page>(current_page_index);
        int page_interval_ms = SetPageInterval(current_page);

        // Update the current page's content until its interval is over
        uint32_t page_start_time = millis();
        while(millis() - page_start_time < page_interval_ms) {
            SetPage(screen, current_page);
            vTaskDelay(pdMS_TO_TICKS(screen_update_rate_ms));
        }

        // Move to the next page
        current_page_index = (current_page_index + 1) % NumberPages;
    }
}
