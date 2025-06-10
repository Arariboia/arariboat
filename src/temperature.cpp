#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "DallasTemperature.h" // For the DS18B20 temperature probes.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "data_temperatures.h" // Header file for temperature-related functions and definitions.

//TODO: Create a heating system to calibrate the temperature probes using thermocouples
//TODO: Make attaching new probes more dynamic instead of hardcoded. Maybe use a config file to store the addresses of the probes.

/// @brief Auxiliary function to print the 8-byte address of a Dallas Thermal Probe to the serial port
/// @param device_address 
static void PrintProbeAddress(DeviceAddress device_address) {

    uint8_t device_address_length = 8; // The length of the device address is 8 bytes
    for (uint8_t i = 0; i < device_address_length; i++) { // Loop through each byte in the eight-byte address
        if (device_address[i] < 16) Serial.print("0"); // If byte is less than 0x10, add a leading zero to maintain 2-digit format
        Serial.print(device_address[i], HEX);
    }
    Serial.printf("\n");
}

/// @brief Scans for Dallas Thermal Probes and prints their addresses to the serial port
/// After adding a new probe, run this function to find the address of the probe. Then hardcode the address into the program
/// for faster performance.
/// @param probes 
void ScanProbeAddresses(DallasTemperature &probes) {
    probes.begin(); // Scan for devices on the OneWire bus.

    char message[256];
    memset(message, 0, sizeof(message));
    
    #ifdef DEBUG
    if (probes.getDeviceCount() == 0) {
        return;
    }

    snprintf(message, sizeof(message), "\n[TEMPERATURE]Found %d probes\n", probes.getDeviceCount());
    for (uint8_t i = 0; i < probes.getDeviceCount(); i++) {
        DeviceAddress device_address;
        if (!probes.getAddress(device_address, i)) {
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "[TEMPERATURE]Unable to find address for Device %d\n", i);
        } else {
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "[TEMPERATURE]Device %d Address: ", i);
            for (uint8_t j = 0; j < 8; j++) {
                snprintf(message + strlen(message), sizeof(message) - strlen(message), "%02x", device_address[j]);
            }
            snprintf(message + strlen(message), sizeof(message) - strlen(message), "\n");
        }
    }

    DEBUG_PRINTF("%s", message);
    #endif
}

static void commandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (STRINGS_ARE_EQUAL(command, "temperature")) {
        Serial.printf("\n[Temperature]: Scanning for new probes\n");
        ScanProbeAddresses(*((DallasTemperature*)handler_args));
    }
}

static void PrintDebugTemperature(float battery_left, float battery_right, float mppt) {
    
    static unsigned long last_print_time = 0;
    if (millis() - last_print_time < 20000) {
        return;
    }
    last_print_time = millis();

    DEBUG_PRINTF("\n[Temperature]Battery Left: %f\n", battery_left);
    DEBUG_PRINTF("\n[Temperature]Battery Right: %f\n", battery_right);
    DEBUG_PRINTF("\n[Temperature]MPPT: %f\n", mppt);
}

void PrintAllDetectedTemperatures(DallasTemperature& probes) {
    probes.begin();
    int device_count = probes.getDeviceCount();
    if (device_count == 0) {
        Serial.println("[Temperature] No sensors detected.");
        return;
    }

    probes.requestTemperatures();

    for (uint8_t i = 0; i < device_count; i++) {
        DeviceAddress addr;
        if (probes.getAddress(addr, i)) {
            float tempC = probes.getTempC(addr);
            Serial.print("[Temperature] Sensor ");
            Serial.print(i);
            Serial.print(" (");
            for (uint8_t j = 0; j < 8; j++) {
                if (addr[j] < 16) Serial.print("0");
                Serial.print(addr[j], HEX);
            }
            Serial.print("): ");
            if (tempC == DEVICE_DISCONNECTED_C) {
                Serial.println("Disconnected or error.");
            } else {
                Serial.print(tempC);
                Serial.println(" °C");
            }
        } else {
            Serial.printf("[Temperature] Unable to get address for sensor %d\n", i);
        }
    }
}

void TemperatureTask(void* parameter) {

    constexpr uint8_t pin_temperature = PIN_TEMPERATURE; // GPIO used for OneWire communication
    
    OneWire one_wire_device(pin_temperature); // Setup a one_wire_device instance to communicate with any devices that use the OneWire protocol
    DallasTemperature probes(&one_wire_device); // Pass our one_wire_device reference to Dallas Temperature sensor, which uses the OneWire protocol.
  
    #define S1 0x28, 0x37, 0x1E, 0x04, 0x00, 0x00, 0x00, 0xA6
    #define S2 0x28, 0xFF, 0x25, 0x61, 0xA3, 0x16, 0x05, 0x16
    #define S3 0x28, 0xFF, 0x64, 0x1F, 0x4D, 0xB8, 0xFE, 0xDA
    #define S4 0x28, 0xCF, 0x67, 0x49, 0xF6, 0x4D, 0x3C, 0xC5
    #define S5 0x28, 0xFF, 0x6F, 0x10, 0xA0, 0x16, 0x03, 0x5C

    //Each probe has a unique 8-byte address. Use the scanIndex method to initially find the addresses of the probes. 
    //Then hardcode the addresses into the program. This is done to avoid the overhead of scanning for the addresses every time the function is called.
    //You should then physically label the probes with tags or stripes as to differentiate them.
    DeviceAddress thermal_probe_mppt_left = {S1};
    DeviceAddress thermal_probe_mppt_right = {S2};
    DeviceAddress thermal_probe_battery_left = {S3};
    DeviceAddress thermal_probe_battery_right = {S4};

    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, COMMAND_BASE, ESP_EVENT_ANY_ID, commandCallback, &probes);

    while (true) {
        // PrintAllDetectedTemperatures(probes); // Generic function to print temperatures from any connected sensor
        
        probes.begin();
        int device_count = probes.getDeviceCount();
        if (device_count == 0) {
            Serial.println("[Temperature] No sensors detected.");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        probes.requestTemperatures();
        float temperature_mppt_left = probes.getTempC(thermal_probe_mppt_left);
        float temperature_mppt_right = probes.getTempC(thermal_probe_mppt_right);
        float temperature_battery_left = probes.getTempC(thermal_probe_battery_left);
        float temperature_battery_right = probes.getTempC(thermal_probe_battery_right);
        
        
        if (temperature_mppt_left != DEVICE_DISCONNECTED_C) {
            temperature_mppt_left = LinearCorrection(temperature_mppt_left, 1.0f, 0.0f);
        }

        if (temperature_mppt_right != DEVICE_DISCONNECTED_C) {
            temperature_mppt_right = LinearCorrection(temperature_mppt_right, 1.0f, 0.0f);
        }

        if (temperature_battery_left != DEVICE_DISCONNECTED_C) {
            temperature_battery_left = LinearCorrection(temperature_battery_left, 1.0f, 0.0f);
        }

        PrintDebugTemperature(temperature_mppt_left, temperature_mppt_right, temperature_battery_left);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

