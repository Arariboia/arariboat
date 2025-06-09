#include "mppt.h"

// --- Constants for Modbus Registers ---
const uint16_t REG_ELECTRICAL_DATA  = 0x3100; // PV/Battery voltage, current
const uint16_t REG_STATUS_DATA      = 0x3200; // Battery and equipment status flags
const uint16_t REG_NET_BATT_CURRENT = 0x331B; // Net battery current (32-bit)


// Defines the order in which registers are polled.
// Note the function names have been updated for clarity.
const MPPTController::RegistryPollFunc MPPTController::_registryPollFunctions[] = {
    &MPPTController::pollElectricalData,
    &MPPTController::pollNetBatteryCurrent,
    &MPPTController::pollStatusData
};


// --- Constructor and Setup ---
MPPTController::MPPTController(Stream& serial, uint8_t slaveId)
    : _serial(serial), _slaveId(slaveId) {
    // Zero-initialize the data struct on creation.
    memset(&_data, 0, sizeof(_data));
}

void MPPTController::begin() {
    _node.begin(_slaveId, _serial);
    _node.idle([]() {
        vTaskDelay(pdMS_TO_TICKS(10)); // Allow other tasks to run
    });
}


// --- Main Update Loop ---
void MPPTController::update() {
    if (millis() - _lastPollTime < POLL_INTERVAL_MS) {
        return;
    }
    _lastPollTime = millis();

    bool success = (this->*_registryPollFunctions[_currentRegistryIndex])();

    if (success) {
        // Update the timestamp only on a successful read.
        _data.timestamp_ms = millis();
    }

    _currentRegistryIndex++;
    if (_currentRegistryIndex >= (sizeof(_registryPollFunctions) / sizeof(_registryPollFunctions[0]))) {
        _currentRegistryIndex = 0;
    }
}

// --- Private Data Polling Methods ---

bool MPPTController::pollElectricalData() {
    // Read 6 registers to get PV Voltage/Current and Battery Voltage.
    // We skip the power registers, but need to read past them.
    uint8_t result = _node.readInputRegisters(REG_ELECTRICAL_DATA, 8);
    if (result != _node.ku8MBSuccess) {
        Serial.printf("Error: Failed to read Electrical Data (0x%X), code: %d\n", REG_ELECTRICAL_DATA, result);
        return false;
    }
    // Store raw integer values directly, no conversion yet.
    _data.electrical.pv_voltage_cV = _node.getResponseBuffer(0x00);
    _data.electrical.pv_current_cA = _node.getResponseBuffer(0x01);
    _data.electrical.battery_voltage_cV = _node.getResponseBuffer(0x04);
    return true;
}

bool MPPTController::pollNetBatteryCurrent() {
    uint8_t result = _node.readInputRegisters(REG_NET_BATT_CURRENT, 2);
    if (result != _node.ku8MBSuccess) {
        Serial.printf("Error: Failed to read Net Battery Current (0x%X), code: %d\n", REG_NET_BATT_CURRENT, result);
        return false;
    }
    // Combine two 16-bit registers into a 32-bit value and store it raw.
    _data.electrical.battery_current_cA = (int32_t)((uint32_t)_node.getResponseBuffer(0x01) << 16 | _node.getResponseBuffer(0x00));
    return true;
}

bool MPPTController::pollStatusData() {
    uint8_t result = _node.readInputRegisters(REG_STATUS_DATA, 2);
    if (result != _node.ku8MBSuccess) {
        Serial.printf("Error: Failed to read Status Data (0x%X), code: %d\n", REG_STATUS_DATA, result);
        return false;
    }
    _data.state.battery_status = _node.getResponseBuffer(0x00);
    _data.state.charging_equipment_status = _node.getResponseBuffer(0x01);
    return true;
}


// --- Public "On-Demand" Printing Methods ---

void MPPTController::printHumanReadableElectricalData() const {
    Serial.println("--- MPPT Electrical Data ---");
    // Convert to float ONLY for printing
    Serial.printf("PV Voltage:            %.2f V\n", _data.electrical.pv_voltage_cV / 100.0f);
    Serial.printf("PV Current:            %.2f A\n", _data.electrical.pv_current_cA / 100.0f);
    Serial.printf("Battery Voltage:       %.2f V\n", _data.electrical.battery_voltage_cV / 100.0f);
    Serial.printf("Net Battery Current:   %.2f A\n", _data.electrical.battery_current_cA / 100.0f);
    Serial.println("----------------------------");
}

void MPPTController::printHumanReadableBatteryStatus() const {
    Serial.println("--- Battery Status ---");
    // Decode D3-D0 for voltage status
    switch (_data.state.battery_status & 0x0F) {
        case 0b0000: Serial.println("Voltage: Normal"); break;
        case 0b0001: Serial.println("Voltage: Overvolt"); break;
        case 0b0010: Serial.println("Voltage: Undervolt"); break;
        case 0b0011: Serial.println("Voltage: Low Volt Disconnect"); break;
        case 0b0100: Serial.println("Voltage: Fault"); break;
        default:     Serial.println("Voltage: Unknown"); break;
    }
    Serial.printf("Internal Resistance: %s\n", isBitSet(_data.state.battery_status, 8) ? "Abnormal" : "Normal");
    if (isBitSet(_data.state.battery_status, 15)) {
        Serial.println("Warning: Wrong identification for rated voltage!");
    }
    Serial.println("----------------------");
}

void MPPTController::printHumanReadableEquipmentStatus() const {
    Serial.println("--- Equipment Status ---");
    Serial.printf("Overall Status: %s / %s\n", isBitSet(_data.state.charging_equipment_status, 0) ? "Running" : "Standby", isBitSet(_data.state.charging_equipment_status, 1) ? "FAULT" : "Normal");

    switch ((_data.state.charging_equipment_status >> 2) & 0b11) {
        case 0b00: Serial.println("Charging: None"); break;
        case 0b01: Serial.println("Charging: Float"); break;
        case 0b10: Serial.println("Charging: Boost"); break;
        case 0b11: Serial.println("Charging: Equalization"); break;
    }

    if (isBitSet(_data.state.charging_equipment_status, 4))  Serial.println("Fault: PV Input Short");
    
    switch ((_data.state.charging_equipment_status >> 14) & 0b11) {
        case 0b00: Serial.println("PV Input: Normal"); break;
        case 0b01: Serial.println("PV Input: No Power Connected"); break;
        case 0b10: Serial.println("PV Input: High Voltage"); break;
        case 0b11: Serial.println("PV Input: Voltage Error"); break;
    }
    Serial.println("------------------------");
}


// Task function to handle all MPPT communication.
void mppt_task(void *parameters) {

        // --- Configuration ---
    constexpr gpio_num_t MPPT_RX_PIN = GPIO_NUM_22;
    constexpr gpio_num_t MPPT_TX_PIN = GPIO_NUM_23;
    constexpr int MODBUS_SLAVE_ID = 1;
    const unsigned long PRINT_INTERVAL_MS = 10000; // How often to print data to Serial

    // Instantiate the controller object, passing the hardware serial port and slave ID.
    MPPTController mppt(Serial2, MODBUS_SLAVE_ID);

    unsigned long lastPrintTime = 0;


    // Start the hardware serial for Modbus communication
    Serial2.begin(115200, SERIAL_8N1, MPPT_RX_PIN, MPPT_TX_PIN);

    // Initialize the controller
    mppt.begin();

    Serial.println("[MPPT] Setup Complete. Entering main loop...");
    while (true) {
        // The update() method handles all Modbus communication and internal timing.
        mppt.update();

        // This block handles printing the data at a fixed interval.
        if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
            lastPrintTime = millis();

            // Get a reference to the latest raw data
            const mppt_data_t& data = mppt.getData();

            // Check if data is recent. If the controller is offline, the timestamp stops updating.
            // We'll consider data stale if it's older than 3 polling cycles.
            if (data.timestamp_ms > 0 && (millis() - data.timestamp_ms < (POLL_INTERVAL_MS * 3))) {
                Serial.println(); // Add a blank line for readability
                // Call the "on-demand" conversion and printing methods
                mppt.printHumanReadableElectricalData();
                mppt.printHumanReadableBatteryStatus();
                mppt.printHumanReadableEquipmentStatus();
                Serial.printf("MPPT Data at %lu ms:\n", data.timestamp_ms);
            } else {
                Serial.println("[Task] Waiting for data from MPPT controller...");
            }
        }

        // Yield to other tasks.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}