#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Adafruit_ADS1X15.h" // 16-bit high-linearity with programmable gain amplifier Analog-Digital Converter for measuring current and voltage.
#include <SPI.h> // Required for the ADS1115 ADC.
#include <Wire.h> // Required for the ADS1115 ADC and communication with the LoRa board and memory chip.
#include "Utilities.hpp" // Custom utility macros and functions. (Assuming LinearCorrection, STRINGS_ARE_EQUAL, DEBUG_PRINTF are here)
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include "data_instrumentation.h" // Data structs for instrumentation data
#include "INA226.h" // Library for the INA226 current and voltage sensor.

/* ADC Description Comments */
// (Original comments about ADS1115 principles and usage are kept but omitted here for brevity)

typedef Adafruit_ADS1115 ADS1115; // Alias for the ADS1115 class.

// --- Calibration Data Management ---
struct __attribute__((packed)) LinearCalibration {
    float slope;
    float intercept;
};

//Packed structure to prevent memory padding issues
struct __attribute__((packed)) AllCalibrationData {
    uint8_t version;
    uint8_t isValidFlag; // e.g., 0xAA for valid
    LinearCalibration battery_current;
    LinearCalibration motor_left_current;
    LinearCalibration motor_right_current;
    LinearCalibration mppt_current;
    float irradiance_sensitivity_millivolts; // e.g., 22.4mV for 1000 W/m^2
};

// Current version of the calibration data structure
const uint8_t CALIBRATION_DATA_VERSION = 1;
const uint8_t CALIBRATION_VALID_MARKER = 0xAA;
const uint16_t EEPROM_CALIBRATION_BASE_ADDR = 0; // Start address in EEPROM for calibration data

// Global struct to hold current calibration values
AllCalibrationData current_calibrations;

// Default calibration values (used if EEPROM is empty/invalid, and for GAIN_ONE on currentsAdc)
const LinearCalibration DEFAULT_CAL_BATTERY_CURRENT   = {0.013063f, -227.935685f};
const LinearCalibration DEFAULT_CAL_MOTOR_L_CURRENT = {0.004162f, -54.649315f};
const LinearCalibration DEFAULT_CAL_MOTOR_R_CURRENT = {0.004176f, -54.880628f};
const LinearCalibration DEFAULT_CAL_MPPT_CURRENT    = {0.004146f, -54.474844f};
const float DEFAULT_IRRADIANCE_SENSITIVITY_MILLIVOLTS = 22.4f; // mV reading for 1000 W/m^2

constexpr uint8_t memory_chip_address = 0x50; // I2C Address of the memory chip

uint8_t read_byte_from_memory_raw(uint8_t address_offset) {
    Wire.beginTransmission(memory_chip_address);
    Wire.write(EEPROM_CALIBRATION_BASE_ADDR + address_offset);
    Wire.endTransmission(false);
    Wire.requestFrom(memory_chip_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    DEBUG_PRINTF("[ERROR] Failed to read byte from memory at offset %u\n", address_offset);
    return 0;
}

bool write_byte_to_memory_raw(uint8_t address_offset, uint8_t value) {
    Wire.beginTransmission(memory_chip_address);
    Wire.write(EEPROM_CALIBRATION_BASE_ADDR + address_offset);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

void load_calibrations_from_memory(AllCalibrationData& cal_data_struct) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&cal_data_struct);
    for (size_t i = 0; i < sizeof(AllCalibrationData); ++i) {
        ptr[i] = read_byte_from_memory_raw(i);
    }
    DEBUG_PRINTF("[Calibration] Loaded %zu bytes from memory.\n", sizeof(AllCalibrationData));
}

void save_calibrations_to_memory(const AllCalibrationData& cal_data_struct) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&cal_data_struct);
    bool success = true;
    for (size_t i = 0; i < sizeof(AllCalibrationData); ++i) {
        if (!write_byte_to_memory_raw(i, ptr[i])) {
            success = false;
            DEBUG_PRINTF("[ERROR] Failed to write byte to memory at offset %zu\n", i);
            // break; 
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid overwhelming the I2C bus
    }
    if (success) {
        DEBUG_PRINTF("[Calibration] Saved %zu bytes to memory.\n", sizeof(AllCalibrationData));
    } else {
        DEBUG_PRINTF("[ERROR] Failed to save complete calibration data to memory.\n");
    }
}

void initialize_default_calibrations(AllCalibrationData& cal_data_struct) {
    DEBUG_PRINTF("[Calibration] Initializing with default calibration values.\n");
    cal_data_struct.version = CALIBRATION_DATA_VERSION;
    cal_data_struct.isValidFlag = CALIBRATION_VALID_MARKER;
    cal_data_struct.battery_current = DEFAULT_CAL_BATTERY_CURRENT;
    cal_data_struct.motor_left_current = DEFAULT_CAL_MOTOR_L_CURRENT;
    cal_data_struct.motor_right_current = DEFAULT_CAL_MOTOR_R_CURRENT;
    cal_data_struct.mppt_current = DEFAULT_CAL_MPPT_CURRENT;
    cal_data_struct.irradiance_sensitivity_millivolts = DEFAULT_IRRADIANCE_SENSITIVITY_MILLIVOLTS;
}
// --- End Calibration Data Management ---


static void commandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    const char* command = (const char*)event_data;

    if (STRINGS_ARE_EQUAL(command, "adc")) {
        ADS1115* adc = (ADS1115*)handler_args; // This will be currentsAdc

        Serial.printf("\n[Instrumentation][CmdCallback] Reading ADC values (using specific gains & cal factors)\n");
        // Gains and calibration factors are for GAIN_ONE, matching defaults.
        adc->setGain(GAIN_ONE);
        float battery_current_cmd = LinearCorrection(adc->readADC_SingleEnded(0), 0.013063f, -227.935685f);

        adc->setGain(GAIN_ONE);
        float current_motor_left_cmd = LinearCorrection(adc->readADC_SingleEnded(1), 0.004162f, -54.649315f);

        adc->setGain(GAIN_ONE);
        float current_motor_right_cmd = LinearCorrection(adc->readADC_SingleEnded(2), 0.004176f, -54.880628f);

        adc->setGain(GAIN_ONE);
        float current_mppt_cmd = LinearCorrection(adc->readADC_SingleEnded(3), 0.004146f, -54.474844f);

        Serial.printf("\n[Instrumentation][CmdCallback] Battery current: %.2fA (Gain ONE)\n"
                      "[Instrumentation][CmdCallback] Port current: %.2fA (Gain ONE)\n"
                      "[Instrumentation][CmdCallback] Starboard current: %.2fA (Gain ONE)\n"
                      "[Instrumentation][CmdCallback] MPPT current: %.2fA (Gain ONE)\n",
                      battery_current_cmd, current_motor_left_cmd, current_motor_right_cmd, current_mppt_cmd);
    } else if (STRINGS_ARE_EQUAL(command, "savecal")) {
        DEBUG_PRINTF("[Calibration] Command 'savecal' received. Saving current operational calibrations to memory.\n");
        save_calibrations_to_memory(current_calibrations);
    } else if (STRINGS_ARE_EQUAL(command, "loaddefcal")) {
        DEBUG_PRINTF("[Calibration] Command 'loaddefcal' received. Loading defaults and saving to memory.\n");
        initialize_default_calibrations(current_calibrations);
        save_calibrations_to_memory(current_calibrations);
    }
}

float CalculateIrradiance(float solar_panel_voltage_millivolts, float sensitivity_for_1000_wm2_millivolts) {
    // sensitivity_for_1000_wm2_millivolts is the voltage reading (e.g., 22.4mV) expected for an irradiance of 1000 W/m^2.
    if (sensitivity_for_1000_wm2_millivolts < 0.001f && sensitivity_for_1000_wm2_millivolts > -0.001f) { // Avoid division by zero or very small numbers
        return 0.0f; // Or handle error appropriately
    }
    float irradiance = (solar_panel_voltage_millivolts / sensitivity_for_1000_wm2_millivolts) * 1000.0f;
    return irradiance;
}

bool EndsWithNewline(const char* str) {
    if (str == nullptr) return false;
    size_t len = strlen(str);
    return len > 0 && str[len - 1] == '\n';
}

INA226 aux_battery_monitor(0x40);

bool i2c_scan() {
    bool device_found = false;
    Serial.println("Scanning I2C bus...");
    for (uint8_t i = 1; i < 127; i++) {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0) {
            Serial.printf("Found device at address: 0x%02X (%d)\n", i, i);
            device_found = true;
        }
    }
    Serial.println("I2C scan complete.");
    return device_found;
}

void get_readings_ina226() {
    float busVoltage = aux_battery_monitor.getBusVoltage();
    float current = aux_battery_monitor.getCurrent_mA();
    float power = aux_battery_monitor.getPower_mW();
    float shuntVoltage = aux_battery_monitor.getShuntVoltage_mV();
    Serial.printf("INA226: Bus Voltage: %.3f V\tShunt Voltage: %.3f mV\tCurrent: %.3f mA\tPower: %.3f mW\n", busVoltage, shuntVoltage, current, power);
}

void InstrumentationTask(void* parameter) {
    
    constexpr gpio_num_t i2c_scl = GPIO_NUM_19; // ESP32 SCL Pin
    constexpr gpio_num_t i2c_sda = GPIO_NUM_21; // ESP32 SDA Pin
    Wire.begin(i2c_sda, i2c_scl);

    // i2c_scan(); // Uncomment if you want to scan I2C devices on startup

    ADS1115 currentsAdc; // For battery, motors, MPPT currents
    ADS1115 voltagesAdc; // For irradiance sensor voltage, etc.

    constexpr uint8_t currents_adc_address = 0x48; // Primary ADC for currents
    constexpr uint8_t voltages_adc_address = 0x49; // Secondary ADC (Update if different)
    
    bool is_currents_adc_initialized = false;
    bool is_voltages_adc_initialized = false;
    
    DEBUG_PRINTF("[Calibration] Attempting to load calibrations from memory chip...\n");
    load_calibrations_from_memory(current_calibrations);

    if (current_calibrations.isValidFlag != CALIBRATION_VALID_MARKER || current_calibrations.version != CALIBRATION_DATA_VERSION) {
        DEBUG_PRINTF("[Calibration] Invalid or outdated data in memory. Loading defaults and saving.\n");
        initialize_default_calibrations(current_calibrations);
        save_calibrations_to_memory(current_calibrations);
    } else {
        DEBUG_PRINTF("[Calibration] Successfully loaded calibrations from memory.\n");
    }
    
    // aux_battery_monitor.begin(); // Initialize INA226 if used

    // esp_event_handler_register_with(eventLoop, COMMAND_BASE, ESP_EVENT_ANY_ID, commandCallback, &currentsAdc);
    // Adapt serial command handling if not using ESP-IDF events.

    DEBUG_PRINTF("[Instrumentation] Starting main measurement loop.\n");

    while (true) {
        char instrumentation_debug_buffer[512]; 
        memset(instrumentation_debug_buffer, 0, sizeof(instrumentation_debug_buffer));
        size_t buffer_current_len = 0;

        // --- Attempt to initialize ADCs if not already done ---
        if (!is_currents_adc_initialized) {
            if (currentsAdc.begin(currents_adc_address)) {
                DEBUG_PRINTF("\n[ADS]Currents ADC successfully initialized at address 0x%x\n", currents_adc_address);
                currentsAdc.setDataRate(RATE_ADS1115_16SPS);
                currentsAdc.setGain(GAIN_ONE); 
                is_currents_adc_initialized = true;
            } else {
                 DEBUG_PRINTF("[ADS]Currents ADC init attempt failed for 0x%x (will retry).\n", currents_adc_address);
            }
        }

        if (!is_voltages_adc_initialized) {
            if (voltagesAdc.begin(voltages_adc_address)) {
                DEBUG_PRINTF("\n[ADS]Voltages ADC successfully initialized at address 0x%x\n", voltages_adc_address);
                voltagesAdc.setDataRate(RATE_ADS1115_16SPS);
                voltagesAdc.setGain(GAIN_ONE); // Gain for voltage measurements (e.g., irradiance sensor)
                is_voltages_adc_initialized = true;
            } else {
                 DEBUG_PRINTF("[ADS]Voltages ADC init attempt failed for 0x%x (will retry).\n", voltages_adc_address);
            }
        }

        // --- Perform measurements based on ADC status ---
        if (is_currents_adc_initialized) {
            // currentsAdc.setGain(GAIN_ONE); // Ensure gain is set if it could be changed, set during init
            float battery_current = LinearCorrection(currentsAdc.readADC_SingleEnded(0), 
                                                     current_calibrations.battery_current.slope, 
                                                     current_calibrations.battery_current.intercept);
            float current_motor_left = LinearCorrection(currentsAdc.readADC_SingleEnded(1), 
                                                        current_calibrations.motor_left_current.slope, 
                                                        current_calibrations.motor_left_current.intercept);
            float current_motor_right = LinearCorrection(currentsAdc.readADC_SingleEnded(2), 
                                                         current_calibrations.motor_right_current.slope, 
                                                         current_calibrations.motor_right_current.intercept);
            float current_mppt = LinearCorrection(currentsAdc.readADC_SingleEnded(3), 
                                                  current_calibrations.mppt_current.slope, 
                                                  current_calibrations.mppt_current.intercept);

            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len, 
                                     sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                     "%s[Instrumentation]Battery current: %.2fA\n"
                                     "[Instrumentation]Left motor current: %.2fA\n"
                                     "[Instrumentation]Right motor current: %.2fA\n"
                                     "[Instrumentation]MPPT current: %.2fA\n",
                                     EndsWithNewline(instrumentation_debug_buffer) ? "" : "\n",
                                     battery_current, current_motor_left, current_motor_right, current_mppt);
        } else {
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                     sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                     "%s[Instrumentation]Currents ADC not initialized.\n",
                                     EndsWithNewline(instrumentation_debug_buffer) ? "" : "\n");
        }

        if (is_voltages_adc_initialized) {
            int16_t raw_adc_irradiance_ch = voltagesAdc.readADC_SingleEnded(0); // Assuming channel 0 for irradiance voltage
            float solar_panel_voltage_millivolts = voltagesAdc.computeVolts(raw_adc_irradiance_ch) * 1000.0f;
            float irradiance = CalculateIrradiance(solar_panel_voltage_millivolts, current_calibrations.irradiance_sensitivity_millivolts);
            
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                     sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                     "[Instrumentation]Irradiance: %.0f W/m^2 (Raw: %d, Sen_Voltage: %.1fmV)\n",
                                     irradiance, raw_adc_irradiance_ch, solar_panel_voltage_millivolts);
        } else {
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                     sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                     "[Instrumentation]Voltages ADC not initialized.\n");
        }
        
        if (buffer_current_len > 0) {
            DEBUG_PRINTF("%s", instrumentation_debug_buffer);
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Main loop delay
    }
}
