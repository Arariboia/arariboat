// THIS IS A FILE FOR QUICK BENCH TESTS

#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include <Wire.h>

// Alias for the ADS1115 class
using ADS1115 = Adafruit_ADS1115;

// GPIO pins for I2C
constexpr gpio_num_t I2C_SCL_PIN = PIN_SCL;
constexpr gpio_num_t I2C_SDA_PIN = PIN_SDA;

// --- Configuration Structure for each Channel ---
// This struct holds all the settings for a single ADC channel.
struct ChannelConfig {
    float slope;      // Calibration slope
    float intercept;  // Calibration intercept
    adsGain_t gain;   // ADC gain setting
};

// --- Channel Configuration Array ---
// An array of configurations, one for each of the 4 channels.
// This makes it easy to change settings for any channel in one place.
ChannelConfig channel_configs[4] = {
    // Channel 0 (A0)
    { 1.0f, 0.0f, GAIN_ONE },
    // Channel 1 (A1)
    { 1.0f, 0.0f, GAIN_ONE },
    // Channel 2 (A2)
    { 1.0f, 0.0f, GAIN_ONE },
    // Channel 3 (A3)
    { 1.0f, 0.0f, GAIN_ONE }
};

// Create an instance of the ADS1115
ADS1115 adc;

// Helper function to convert gain enum to a human-readable string
const char* gainToString(adsGain_t gain) {
    switch (gain) {
        case GAIN_TWOTHIRDS: return "2/3X";
        case GAIN_ONE:       return "1X";
        case GAIN_TWO:       return "2X";
        case GAIN_FOUR:      return "4X";
        case GAIN_EIGHT:     return "8X";
        case GAIN_SIXTEEN:   return "16X";
        default:             return "UNK";
    }
}

void InstrumentationTask(void *parameter) {
    // A small delay to allow the serial monitor to connect
    vTaskDelay(pdMS_TO_TICKS(1000));

    Serial.println("Starting instrumentation calibrator...");

    // Initialize the I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // --- Scan for ADC on I2C bus ---
    uint8_t found_address = 0;
    Serial.println("Scanning for ADS1115 on addresses 0x48 to 0x4B... Will retry until found.");
    
    while (found_address == 0) {
        for (uint8_t addr = 0x48; addr <= 0x4B; addr++) {
            if (adc.begin(addr)) {
                found_address = addr;
                Serial.printf("ADC found at address 0x%X\n", found_address);
                break; // Exit the inner for loop
            }
        }
        
        if (found_address == 0) {
            // Wait a bit before retrying the scan
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }


    // Configure the ADC data rate. Gain is now set per-channel inside the loop.
    adc.setDataRate(RATE_ADS1115_860SPS);

    Serial.println("ADC initialized successfully. Starting measurements.");
    
    // Variables to hold the readings
    int16_t raw_adc_values[4];
    float converted_values[4];
    float voltage_values[4];

    // Timing for printing output every second
    unsigned long last_print_time = 0;
    const unsigned long print_interval_ms = 1000;

    while (true) {
        // --- Data Acquisition Loop (runs as fast as possible) ---
        // This loop now iterates through each channel, applying its specific configuration.
        for (int i = 0; i < 4; i++) {
            // Set the gain for the current channel based on its configuration
            adc.setGain(channel_configs[i].gain);

            // Read the raw ADC value
            raw_adc_values[i] = adc.readADC_SingleEnded(i);

            // Compute the voltage using the library function
            voltage_values[i] = adc.computeVolts(raw_adc_values[i]);

            // Apply the linear correction using the config for the current channel
            converted_values[i] = (static_cast<float>(raw_adc_values[i]) * channel_configs[i].slope) + channel_configs[i].intercept;
        }

        // --- Terminal Output Section (emits data every second) ---
        unsigned long current_time = millis();
        if (current_time - last_print_time >= print_interval_ms) {
            last_print_time = current_time; // Update the last print time

            // Get the time since boot in seconds
            float timestamp_s = current_time / 1000.0f;

            // Print a clear, human-readable header
            Serial.printf("--- Timestamp: %.2f s ---\n", timestamp_s);
            
            // Print the data for each channel in a loop
            for (int i = 0; i < 4; i++) {
                Serial.printf("  A%d: %.3f (raw: %d) (Voltage: %.3f V) [GAIN %s]\n",
                    i,
                    converted_values[i],
                    raw_adc_values[i],
                    voltage_values[i],
                    gainToString(channel_configs[i].gain)
                );
            }
            Serial.println(); // Add a blank line for readability
        }
        
        // Yield control to the scheduler briefly to prevent starving other tasks
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void setup () {
    // Start the serial communication
    Serial.begin(115200);
    while (!Serial) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for Serial to be ready
    }

    // Create the instrumentation task
    xTaskCreatePinnedToCore(
        InstrumentationTask,   // Task function
        "InstrumentationTask", // Name of the task
        8192,                 // Stack size in words
        nullptr,              // Task input parameter
        1,                    // Priority of the task
        nullptr,              // Task handle
        1                     // Core where the task should run
    );
}

void loop() {
    // The main loop is empty because all the work is done in the InstrumentationTask
    vTaskDelay(pdMS_TO_TICKS(1000)); // Sleep to reduce CPU usage
}

