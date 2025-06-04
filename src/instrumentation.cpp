#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include <SPI.h>
#include <Wire.h>
#include "Utilities.hpp"          // Assuming LinearCorrection, STRINGS_ARE_EQUAL, DEBUG_PRINTF
#include "arariboat/mavlink.h"
#include "data_instrumentation.h"
#include "INA226.h"

typedef Adafruit_ADS1115 ADS1115; //Shorter alias for the ADS1115 class from Adafruit library

// ADC I2C Addresses
constexpr uint8_t currents_adc_address = 0x48;
constexpr uint8_t voltages_adc_address = 0x49;

// INA226 I2C Address for Auxiliary Battery
constexpr uint8_t aux_battery_ina226_address = 0x40;

// EEPROM I2C Addresses (MUST BE UNIQUE AND MATCH HARDWARE)
constexpr uint8_t currents_board_eeprom_address = 0x50;
constexpr uint8_t voltages_board_eeprom_address = 0x51;


// --- Generic Calibration Data Management ---
struct __attribute__((packed)) LinearCalibration {
    float slope;
    float intercept;
};

const uint8_t MAX_CALIBRATION_PAIRS_PER_BOARD = 4;
const uint8_t CALIBRATION_DATA_VERSION = 1;
const uint8_t CALIBRATION_VALID_MARKER = 0xAA;
const uint16_t EEPROM_STRUCT_BASE_ADDR = 0; // Structs stored at the beginning of their respective EEPROMs

struct __attribute__((packed)) BoardCalibrationData {
    uint8_t version;
    uint8_t isValidFlag;
    LinearCalibration calibrations[MAX_CALIBRATION_PAIRS_PER_BOARD];
};

// Global instances for loaded calibrations for each board
BoardCalibrationData currents_board_cal_data;
BoardCalibrationData voltages_board_cal_data; // For the second board, e.g., irradiance

// --- Default Calibration Values ---
// LSB for ADS1115 at GAIN_ONE (±4.096V range / 32767 counts)
const float ADC_LSB_VOLTS_GAIN_ONE = 4.096f / 32767.0f; // Approx. 0.00012500381f V/count

// Default for "Currents" Board (using all 4 pairs)
const BoardCalibrationData DEFAULT_CURRENTS_BOARD_CAL = {
    CALIBRATION_DATA_VERSION, CALIBRATION_VALID_MARKER,
    {
        {0.013063f, -227.935685f},  // Channel 0: Battery Current
        {0.004162f, -54.649315f},   // Channel 1: Motor Left Current
        {0.004176f, -54.880628f},   // Channel 2: Motor Right Current
        {0.004146f, -54.474844f}    // Channel 3: MPPT Current
    }
};

// Default for "Voltages" Board (e.g., Irradiance on Channel 0)
// Irradiance: (RawADC * LSB_V * 1000 / Sensitivity_V_1000W)
// Sensitivity_V_1000W = 22.4mV / 1000 = 0.0224V
// Slope = (ADC_LSB_VOLTS_GAIN_ONE * 1000.0f) / 0.0224f
const float DEFAULT_IRRADIANCE_SLOPE = (ADC_LSB_VOLTS_GAIN_ONE * 1000.0f) / 0.0224f; // Approx 5.580527f

const BoardCalibrationData DEFAULT_VOLTAGES_BOARD_CAL = {
    CALIBRATION_DATA_VERSION, CALIBRATION_VALID_MARKER,
    {
        {DEFAULT_IRRADIANCE_SLOPE, 0.0f}, // Channel 0: Irradiance (W/m^2 from raw ADC)
        {0.0f, 0.0f},                     // Channel 1: Unused/Default
        {0.0f, 0.0f},                     // Channel 2: Unused/Default
        {0.0f, 0.0f}                      // Channel 3: Unused/Default
    }
};

// Generic EEPROM byte read/write and struct load/save functions (remain unchanged from previous version)
uint8_t read_byte_from_eeprom(uint8_t eeprom_i2c_address, uint16_t byte_offset_in_struct) {
    Wire.beginTransmission(eeprom_i2c_address);
    Wire.write((uint8_t)(EEPROM_STRUCT_BASE_ADDR + byte_offset_in_struct)); 
    Wire.endTransmission(false); // Send restart to keep connection active for read
    Wire.requestFrom(eeprom_i2c_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    DEBUG_PRINTF("[ERROR] Failed to read from EEPROM 0x%X at offset %u\n", eeprom_i2c_address, byte_offset_in_struct);
    return 0; // Return 0 on failure
}

bool write_byte_to_eeprom(uint8_t eeprom_i2c_address, uint16_t byte_offset_in_struct, uint8_t value) {
    Wire.beginTransmission(eeprom_i2c_address);
    Wire.write((uint8_t)(EEPROM_STRUCT_BASE_ADDR + byte_offset_in_struct)); // EEPROM memory address
    Wire.write(value); // Data byte
    if (Wire.endTransmission() != 0) { // Returns 0 on success
        DEBUG_PRINTF("[ERROR] Failed to write to EEPROM 0x%X at offset %u value %u\n", eeprom_i2c_address, byte_offset_in_struct, value);
        return false;
    }
    return true;
}

template<typename T>
void load_struct_from_eeprom(T& data_struct, uint8_t eeprom_i2c_address) {
    uint8_t* ptr = reinterpret_cast<uint8_t*>(&data_struct);
    DEBUG_PRINTF("[Calibration] Loading %zu bytes from EEPROM 0x%X...\n", sizeof(T), eeprom_i2c_address);
    for (size_t i = 0; i < sizeof(T); ++i) {
        ptr[i] = read_byte_from_eeprom(eeprom_i2c_address, i);
    }
}

template<typename T>
void save_struct_to_eeprom(const T& data_struct, uint8_t eeprom_i2c_address) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&data_struct);
    bool all_success = true;
    DEBUG_PRINTF("[Calibration] Saving %zu bytes to EEPROM 0x%X...\n", sizeof(T), eeprom_i2c_address);
    for (size_t i = 0; i < sizeof(T); ++i) {
        if (!write_byte_to_eeprom(eeprom_i2c_address, i, ptr[i])) {
            all_success = false;
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for EEPROM write cycle
    }
    if (all_success) {
        DEBUG_PRINTF("[Calibration] Saved struct to EEPROM 0x%X successfully.\n", eeprom_i2c_address);
    } else {
        DEBUG_PRINTF("[ERROR] Failed to save entire struct to EEPROM 0x%X.\n", eeprom_i2c_address);
    }
}

// Specific default initializers for the generic BoardCalibrationData struct
void initialize_default_calibrations_for_board(BoardCalibrationData& data_struct, bool is_currents_board) {
    if (is_currents_board) {
        DEBUG_PRINTF("[Calibration] Initializing Currents Board with default generic calibrations.\n");
        data_struct = DEFAULT_CURRENTS_BOARD_CAL;
    } else { // Assuming it's the voltages board
        DEBUG_PRINTF("[Calibration] Initializing Voltages Board with default generic calibrations.\n");
        data_struct = DEFAULT_VOLTAGES_BOARD_CAL;
    }
}
// --- End Calibration Data Management ---


static void commandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    const char* command = (const char*)event_data;
    // Unpack handler_args if it becomes a struct with both ADCs and cal_loaded flags
    ADS1115* currents_adc_ptr = (ADS1115*)handler_args; // Assuming handler_args is still just currentsAdc for simplicity

    if (STRINGS_ARE_EQUAL(command, "adc")) {
        // This command primarily targets the currentsAdc passed in handler_args
        // For full access, handler_args would need to be a struct with all ADCs and cal_loaded flags
        if (!currents_adc_ptr || !currents_board_cal_data.isValidFlag) { // Basic check
             Serial.printf("[ERROR][CmdCallback] Currents ADC or its calibration not ready for 'adc' command.\n");
             return;
        }
        Serial.printf("\n[Instrumentation][CmdCallback] Reading Currents ADC (0x%X) values (using loaded cal):\n", currents_adc_address);
        currents_adc_ptr->setGain(GAIN_ONE); 
        
        for (uint8_t i = 0; i < MAX_CALIBRATION_PAIRS_PER_BOARD; ++i) {
            float val = LinearCorrection(currents_adc_ptr->readADC_SingleEnded(i),
                                         currents_board_cal_data.calibrations[i].slope,
                                         currents_board_cal_data.calibrations[i].intercept);
            Serial.printf("[CmdCallback] Ch %d: %.2f (Slope: %.6f, Intercept: %.2f)\n",
                          i, val,
                          currents_board_cal_data.calibrations[i].slope,
                          currents_board_cal_data.calibrations[i].intercept);
        }
    } else if (STRINGS_ARE_EQUAL(command, "savecal")) {
        DEBUG_PRINTF("[Calibration] Cmd 'savecal': Saving Currents Board calibrations to EEPROM 0x%X.\n", currents_board_eeprom_address);
        save_struct_to_eeprom(currents_board_cal_data, currents_board_eeprom_address);
        DEBUG_PRINTF("[Calibration] Cmd 'savecal': Saving Voltages Board calibrations to EEPROM 0x%X.\n", voltages_board_eeprom_address);
        save_struct_to_eeprom(voltages_board_cal_data, voltages_board_eeprom_address);
    } else if (STRINGS_ARE_EQUAL(command, "loaddefcal")) {
        DEBUG_PRINTF("[Calibration] Cmd 'loaddefcal': Loading default Currents Board calibrations and saving to EEPROM 0x%X.\n", currents_board_eeprom_address);
        initialize_default_calibrations_for_board(currents_board_cal_data, true);
        save_struct_to_eeprom(currents_board_cal_data, currents_board_eeprom_address);
        
        DEBUG_PRINTF("[Calibration] Cmd 'loaddefcal': Loading default Voltages Board calibrations and saving to EEPROM 0x%X.\n", voltages_board_eeprom_address);
        initialize_default_calibrations_for_board(voltages_board_cal_data, false);
        save_struct_to_eeprom(voltages_board_cal_data, voltages_board_eeprom_address);
    }
}

// CalculateIrradiance function is no longer needed, LinearCorrection will be used directly.

bool EndsWithNewline(const char* str) { /* ... remains the same ... */ 
    if (str == nullptr) return false;
    size_t len = strlen(str);
    return len > 0 && str[len - 1] == '\n';
}

bool i2c_scan() { // ... remains the same ...
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

void InstrumentationTask(void* parameter) {
    
    constexpr gpio_num_t i2c_scl = GPIO_NUM_19;
    constexpr gpio_num_t i2c_sda = GPIO_NUM_21;
    Wire.begin(i2c_sda, i2c_scl); // Initialize I2C bus

    ADS1115 currentsAdc;
    ADS1115 voltagesAdc;
    INA226 aux_battery_monitor(aux_battery_ina226_address); // INA226 for auxiliary battery
    
    bool is_currents_adc_initialized = false;
    bool is_voltages_adc_initialized = false;
    bool is_currents_cal_loaded = false;
    bool is_voltages_cal_loaded = false;
    bool is_aux_battery_monitor_initialized = false; // Flag for INA226
    
    // For ESP-IDF event loop, handler_args might need to be a struct containing pointers to ADCs and cal_loaded flags
    // For simplicity, if commandCallback is called directly from Serial, pass &currentsAdc.
    // esp_event_handler_register_with(eventLoop, COMMAND_BASE, ESP_EVENT_ANY_ID, commandCallback, &currentsAdc);

    DEBUG_PRINTF("[Instrumentation] Starting main measurement loop.\n");

    while (true) {
        char instrumentation_debug_buffer[768]; // Increased buffer size for more data
        memset(instrumentation_debug_buffer, 0, sizeof(instrumentation_debug_buffer));
        size_t buffer_current_len = 0;

        // --- CURRENTS ADC & CALIBRATION ---
        if (!is_currents_adc_initialized) {
            if (currentsAdc.begin(currents_adc_address)) {
                DEBUG_PRINTF("\n[ADS]Currents ADC (0x%X) successfully initialized.\n", currents_adc_address);
                currentsAdc.setDataRate(RATE_ADS1115_16SPS);
                currentsAdc.setGain(GAIN_ONE); 
                is_currents_adc_initialized = true;
            } else { DEBUG_PRINTF("\n[ADS]Currents ADC (0x%X) init failed.\n", currents_adc_address); /* Log failure, non-blocking */ }
        }

        if (is_currents_adc_initialized && !is_currents_cal_loaded) {
            DEBUG_PRINTF("[Calibration] Loading for Currents Board (EEPROM 0x%X).\n", currents_board_eeprom_address);
            load_struct_from_eeprom(currents_board_cal_data, currents_board_eeprom_address);
            if (currents_board_cal_data.isValidFlag != CALIBRATION_VALID_MARKER || currents_board_cal_data.version != CALIBRATION_DATA_VERSION) {
                DEBUG_PRINTF("[Calibration] Invalid/outdated for Currents Board. Loading defaults & saving to EEPROM 0x%X.\n", currents_board_eeprom_address);
                initialize_default_calibrations_for_board(currents_board_cal_data, true); // True for currents board
                save_struct_to_eeprom(currents_board_cal_data, currents_board_eeprom_address);
            } else { DEBUG_PRINTF("[Calibration] Loaded for Currents Board from EEPROM 0x%X.\n", currents_board_eeprom_address); }
            is_currents_cal_loaded = true;
        }

        // --- VOLTAGES ADC & CALIBRATION ---
        if (!is_voltages_adc_initialized) {
            if (voltagesAdc.begin(voltages_adc_address)) {
                DEBUG_PRINTF("\n[ADS]Voltages ADC (0x%X) successfully initialized.\n", voltages_adc_address);
                voltagesAdc.setDataRate(RATE_ADS1115_16SPS);
                voltagesAdc.setGain(GAIN_ONE); // GAIN_ONE for LSB consistency with irradiance cal
                is_voltages_adc_initialized = true;
            } else { DEBUG_PRINTF("\n[ADS]Voltages ADC (0x%X) init failed.\n", voltages_adc_address);/* Log failure, non-blocking */ }
        }
        
        if (is_voltages_adc_initialized && !is_voltages_cal_loaded) {
            DEBUG_PRINTF("[Calibration] Loading for Voltages Board (EEPROM 0x%X).\n", voltages_board_eeprom_address);
            load_struct_from_eeprom(voltages_board_cal_data, voltages_board_eeprom_address);
            if (voltages_board_cal_data.isValidFlag != CALIBRATION_VALID_MARKER || voltages_board_cal_data.version != CALIBRATION_DATA_VERSION) {
                DEBUG_PRINTF("[Calibration] Invalid/outdated for Voltages Board. Loading defaults & saving to EEPROM 0x%X.\n", voltages_board_eeprom_address);
                initialize_default_calibrations_for_board(voltages_board_cal_data, false); // False for voltages board
                save_struct_to_eeprom(voltages_board_cal_data, voltages_board_eeprom_address);
            } else { DEBUG_PRINTF("[Calibration] Loaded for Voltages Board from EEPROM 0x%X.\n", voltages_board_eeprom_address); }
            is_voltages_cal_loaded = true;
        }

        // --- AUXILIARY BATTERY MONITOR (INA226) ---
        if (!is_aux_battery_monitor_initialized) {
            if (aux_battery_monitor.begin()) { // Address was set in constructor
                DEBUG_PRINTF("\n[INA226] Aux Battery Monitor (0x%X) successfully initialized.\n", aux_battery_ina226_address);
                // Configure INA226 (max current, shunt resistance, normalize LSB)
                aux_battery_monitor.setMaxCurrentShunt(13.0f, 0.005f, true); 
                is_aux_battery_monitor_initialized = true;
            } else {
                DEBUG_PRINTF("\n[INA226] Aux Battery Monitor (0x%X) init failed.\n", aux_battery_ina226_address);
                /* Log failure, non-blocking, will retry next loop */
            }
        }

        // --- Perform measurements and build output string ---
        if (is_currents_adc_initialized && is_currents_cal_loaded) {
            // Assuming channel 0-3 for battery, motor L, motor R, MPPT current respectively
            float battery_current     = LinearCorrection(currentsAdc.readADC_SingleEnded(0), currents_board_cal_data.calibrations[0].slope, currents_board_cal_data.calibrations[0].intercept);
            float current_motor_left  = LinearCorrection(currentsAdc.readADC_SingleEnded(1), currents_board_cal_data.calibrations[1].slope, currents_board_cal_data.calibrations[1].intercept);
            float current_motor_right = LinearCorrection(currentsAdc.readADC_SingleEnded(2), currents_board_cal_data.calibrations[2].slope, currents_board_cal_data.calibrations[2].intercept);
            float current_mppt        = LinearCorrection(currentsAdc.readADC_SingleEnded(3), currents_board_cal_data.calibrations[3].slope, currents_board_cal_data.calibrations[3].intercept);

            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len, 
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Currents ADC 0x%X | EEPROM 0x%X]\n"
                                           "  Batt: %.2fA, MotL: %.2fA, MotR: %.2fA, MPPT: %.2fA\n",
                                           (buffer_current_len == 0) ? "" : "\n", // Add newline if not first entry
                                           currents_adc_address, currents_board_eeprom_address,
                                           battery_current, current_motor_left, current_motor_right, current_mppt);
        } else {
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Currents Board (ADC 0x%X / EEPROM 0x%X) not ready.]\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           currents_adc_address, currents_board_eeprom_address);
        }

        if (is_voltages_adc_initialized && is_voltages_cal_loaded) {
            // Assuming Channel 0 of voltagesAdc is for irradiance sensor
            int16_t raw_adc_irradiance_ch = voltagesAdc.readADC_SingleEnded(0);
            // Apply generic LinearCorrection for irradiance
            float irradiance = LinearCorrection(raw_adc_irradiance_ch, 
                                                voltages_board_cal_data.calibrations[0].slope, 
                                                voltages_board_cal_data.calibrations[0].intercept);
            
            // For debugging, calculate and show the intermediate voltage if desired
            float irradiance_sensor_voltage_V = voltagesAdc.computeVolts(raw_adc_irradiance_ch);

            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Voltages ADC 0x%X | EEPROM 0x%X]\n"
                                           "  Irradiance: %.0f W/m^2 (RawADC: %d, Sensor V: %.3fV)\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           voltages_adc_address, voltages_board_eeprom_address,
                                           irradiance, raw_adc_irradiance_ch, irradiance_sensor_voltage_V);
            // You can add readings for calibrations[1] through [3] from voltagesAdc if they are used for other sensors.
        } else {
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Voltages Board (ADC 0x%X / EEPROM 0x%X) not ready.]\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           voltages_adc_address, voltages_board_eeprom_address);
        }
        
        // --- Readings from Auxiliary Battery Monitor (INA226) ---
        if (is_aux_battery_monitor_initialized) {
            float aux_bus_voltage = aux_battery_monitor.getBusVoltage();
            // Apply the specific calibration for current: * 0.786f + 8.48E-3f
            float aux_current = aux_battery_monitor.getCurrent() * 0.786f + 0.00848f; 
            float aux_power = aux_battery_monitor.getPower();
            // float aux_shunt_voltage_mv = aux_battery_monitor.getShuntVoltage_mV(); // Optional for debugging

            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Aux Battery INA226 0x%X]\n"
                                           "  Aux V: %.2fV, Aux I: %.3fA, Aux P: %.2fW\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           aux_battery_ina226_address,
                                           aux_bus_voltage, aux_current, aux_power);
        } else {
            buffer_current_len += snprintf(instrumentation_debug_buffer + buffer_current_len,
                                           sizeof(instrumentation_debug_buffer) - buffer_current_len,
                                           "%s[Aux Battery INA226 (0x%X) not ready.]\n",
                                           (buffer_current_len == 0) ? "" : "\n",
                                           aux_battery_ina226_address);
        }
        
        if (buffer_current_len > 0 && (buffer_current_len < sizeof(instrumentation_debug_buffer))) {
            DEBUG_PRINTF("\n--- Instrumentation Tick ---\n%s--- End Tick ---\n", instrumentation_debug_buffer);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Consolidated delay for the main loop (was 500ms)
    }
}