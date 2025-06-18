#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "DallasTemperature.h" // For the DS18B20 temperature probes.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "data.hpp" // Header file for data structures 
#include "time_manager.h" // Header file for time management tasks.
#include "queues.hpp" // Header file for queue initialization and definitions.

const int16_t SENSOR_DISCONNECTED_CDEGC = INT16_MAX;

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

static void PrintDebugTemperature(float battery_left, float battery_right, float mppt_left, float mppt_right) {
    static unsigned long last_print_time = 0;
    if (millis() - last_print_time < 5000) {
        return;
    }
    last_print_time = millis();

    int16_t battery_left_cdegC  = (battery_left  == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(battery_left  * 100);
    int16_t battery_right_cdegC = (battery_right == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(battery_right * 100);
    int16_t mppt_left_cdegC     = (mppt_left     == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(mppt_left    * 100);
    int16_t mppt_right_cdegC    = (mppt_right    == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(mppt_right   * 100);

    char buffer[256];
    int len = 0;
    len += snprintf(buffer + len, sizeof(buffer) - len, "\n[Temperature]Battery Left: %.2f°C (%d cdegC)\n", battery_left,  battery_left_cdegC);
    len += snprintf(buffer + len, sizeof(buffer) - len, "[Temperature]Battery Right: %.2f°C (%d cdegC)\n", battery_right, battery_right_cdegC);
    len += snprintf(buffer + len, sizeof(buffer) - len, "[Temperature]MPPT Left: %.2f°C (%d cdegC)\n", mppt_left, mppt_left_cdegC);
    len += snprintf(buffer + len, sizeof(buffer) - len, "[Temperature]MPPT Right: %.2f°C (%d cdegC)\n", mppt_right, mppt_right_cdegC);

    DEBUG_PRINTF("%s", buffer);
}

void PrintAllDetectedTemperatures(DallasTemperature& probes) {
    probes.begin();
    int device_count = probes.getDeviceCount();
    if (device_count == 0) {
        Serial.println("[Temperature] No sensors detected.");
        return;
    }

    probes.requestTemperatures();

    char buffer[256];
    size_t len = 0;

    for (uint8_t i = 0; i < device_count; i++) {
        DeviceAddress addr;
        if (probes.getAddress(addr, i)) {
            float tempC = probes.getTempC(addr);
            len += snprintf(buffer + len, sizeof(buffer) - len, "[Temperature] Sensor %d (", i);
            for (uint8_t j = 0; j < 8; j++) {
                len += snprintf(buffer + len, sizeof(buffer) - len, "%s%02X", (j == 0) ? "" : " ", addr[j]);
            }
            len += snprintf(buffer + len, sizeof(buffer) - len, "): ");
            if (tempC == DEVICE_DISCONNECTED_C) {
                len += snprintf(buffer + len, sizeof(buffer) - len, "Disconnected or error.\n");
            } else {
                len += snprintf(buffer + len, sizeof(buffer) - len, "%.2f °C (%d cdegC)\n", tempC, static_cast<int16_t>(tempC * 100));
            }
        } else {
            len += snprintf(buffer + len, sizeof(buffer) - len, "[Temperature] Unable to get address for sensor %d\n", i);
        }
        if (len >= sizeof(buffer) - 64) { // Flush if buffer is nearly full
            Serial.print(buffer);
            len = 0;
            buffer[0] = '\0';
        }
    }
    if (len > 0) {
        Serial.print(buffer);
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
    #define S6 0x28, 0x86, 0x1C, 0x07, 0xD6, 0x01, 0x3C, 0x8C


    //Each probe has a unique 8-byte address. Use the scanIndex method to initially find the addresses of the probes. 
    //Then hardcode the addresses into the program. This is done to avoid the overhead of scanning for the addresses every time the function is called.
    //You should then physically label the probes with tags or stripes as to differentiate them.
    DeviceAddress thermal_probe_mppt_left = {S3};
    DeviceAddress thermal_probe_mppt_right = {S5};
    DeviceAddress thermal_probe_battery_left = {S6};
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

        message_t msg;
        msg.source = DATA_SOURCE_TEMPERATURES; 
        msg.timestamp.time_since_boot_ms = millis();
        msg.timestamp.epoch_seconds = get_epoch_seconds();
        msg.timestamp.epoch_ms = get_epoch_millis();

        auto& temperature = msg.payload.temperature;
        temperature.battery_left_cdegC = (temperature_battery_left == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(temperature_battery_left * 100);
        temperature.battery_right_cdegC = (temperature_battery_right == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(temperature_battery_right * 100);
        temperature.mppt_left_cdegC = (temperature_mppt_left == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(temperature_mppt_left * 100);
        temperature.mppt_right_cdegC = (temperature_mppt_right == DEVICE_DISCONNECTED_C) ? SENSOR_DISCONNECTED_CDEGC : static_cast<int16_t>(temperature_mppt_right * 100);

        // Send the temperature data to the central broker queue
        if (xQueueSend(broker_queue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("[Temperature] Warning: Failed to send temperature data to broker queue.");
        }

        PrintDebugTemperature(temperature_battery_left, temperature_battery_right, temperature_mppt_left, temperature_mppt_right);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

