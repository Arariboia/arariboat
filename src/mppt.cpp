#include "mppt.h"

// --- Constants for Modbus Registers (Self-documenting code) ---
const uint16_t REG_RATED_DATA       = 0x3100; // PV/Battery voltage, current, power
const uint16_t REG_STATUS_DATA      = 0x3200; // Battery and equipment status flags
const uint16_t REG_BATT_CURRENT     = 0x331B; // Net battery current

// --- Polling Configuration ---
const uint32_t POLL_INTERVAL_MS     = 2000; // Poll the next register every 2 seconds

// Defines the order in which registers are polled.
const MPPTController::RegistryPollFunc MPPTController::_registryPollFunctions[] = {
    &MPPTController::pollRatedData,
    &MPPTController::pollBatteryCurrent,
    &MPPTController::pollStatusData
};


// --- Constructor and Setup ---
MPPTController::MPPTController(Stream& serial, uint8_t slaveId)
    : _serial(serial), _slaveId(slaveId) {
    // The ModbusMaster object is initialized via the member initializer list
}

void MPPTController::begin() {
    _node.begin(_slaveId, _serial);
	_node.idle([]() {
		vTaskDelay(pdMS_TO_TICKS(10)); //Allows other tasks to run while waiting for Modbus response
	});
}

// --- Main Update Loop ---
void MPPTController::update() {
    // Check if it's time to poll the next register set
    if (millis() - _lastPollTime < POLL_INTERVAL_MS) {
        return;
    }
    _lastPollTime = millis();

    // Call the current function in the polling sequence
    // The `(this->*_registryPollFunctions[_currentRegistryIndex])()` syntax
    // is how you call a pointer-to-member-function.
    bool success = (this->*_registryPollFunctions[_currentRegistryIndex])();

    if (success) {
        _data.lastUpdateTime = millis();
    }

    // Advance to the next function in the polling sequence for the next update cycle.
    _currentRegistryIndex++;
    if (_currentRegistryIndex >= (sizeof(_registryPollFunctions) / sizeof(_registryPollFunctions[0]))) {
        _currentRegistryIndex = 0; // Wrap around to the beginning
    }
}


// --- Private Data Polling Methods ---

bool MPPTController::pollRatedData() {
    uint8_t result = _node.readInputRegisters(REG_RATED_DATA, 8);
    if (result != _node.ku8MBSuccess) {
        Serial.printf("Error: Failed to read Rated Data (0x%X), code: %d\n", REG_RATED_DATA, result);
        return false;
    }
    _data.pvVoltage = _node.getResponseBuffer(0x00) / 100.0f;
    _data.pvCurrent = _node.getResponseBuffer(0x01) / 100.0f;
    _data.pvPower = (uint32_t(_node.getResponseBuffer(0x03) << 16) | _node.getResponseBuffer(0x02)) / 100.0f;
    _data.batteryVoltage = _node.getResponseBuffer(0x04) / 100.0f;
    _data.batteryChargeCurrent = _node.getResponseBuffer(0x05) / 100.0f;
    _data.batteryChargePower = (uint32_t(_node.getResponseBuffer(0x07) << 16) | _node.getResponseBuffer(0x06)) / 100.0f;
    return true;
}

bool MPPTController::pollBatteryCurrent() {
    uint8_t result = _node.readInputRegisters(REG_BATT_CURRENT, 2);
    if (result != _node.ku8MBSuccess) {
        Serial.printf("Error: Failed to read Battery Current (0x%X), code: %d\n", REG_BATT_CURRENT, result);
        return false;
    }
    // This value is signed, so we must cast to int32_t before division.
    _data.batteryOverallCurrent = int32_t(uint32_t(_node.getResponseBuffer(0x01) << 16) | _node.getResponseBuffer(0x00)) / 100.0f;
    return true;
}

bool MPPTController::pollStatusData() {
    uint8_t result = _node.readInputRegisters(REG_STATUS_DATA, 2);
    if (result != _node.ku8MBSuccess) {
        Serial.printf("Error: Failed to read Status Data (0x%X), code: %d\n", REG_STATUS_DATA, result);
        return false;
    }
    _data.batteryStatus = _node.getResponseBuffer(0x00);
    _data.equipmentStatus = _node.getResponseBuffer(0x01);
    return true;
}


// --- Public Data Printing Methods ---

void MPPTController::printRatedData() const {
    Serial.println("--- MPPT Rated Data ---");
    Serial.printf("PV Voltage:            %.2f V\n", _data.pvVoltage);
    Serial.printf("PV Current:            %.2f A\n", _data.pvCurrent);
    Serial.printf("PV Power:              %.2f W\n", _data.pvPower);
    Serial.printf("Battery Voltage:       %.2f V\n", _data.batteryVoltage);
    Serial.printf("Battery Charge Current: %.2f A\n", _data.batteryChargeCurrent);
    Serial.printf("Battery Charge Power:  %.2f W\n", _data.batteryChargePower);
    Serial.printf("Battery Net Current:   %.2f A\n", _data.batteryOverallCurrent);
    Serial.println("-------------------------");
}

void MPPTController::printBatteryStatus() const {
    Serial.println("--- Battery Status ---");
    // Decode D3-D0 for voltage status
    switch (_data.batteryStatus & 0x0F) {
        case 0b0000: Serial.println("Voltage: Normal"); break;
        case 0b0001: Serial.println("Voltage: Overvolt"); break;
        case 0b0010: Serial.println("Voltage: Undervolt"); break;
        case 0b0011: Serial.println("Voltage: Low Volt Disconnect"); break;
        case 0b0100: Serial.println("Voltage: Fault"); break;
        default:     Serial.println("Voltage: Unknown"); break;
    }
    // Decode D8 for internal resistance
    Serial.printf("Internal Resistance: %s\n", isBitSet(_data.batteryStatus, 8) ? "Abnormal" : "Normal");
    // Decode D15 for rated voltage identification
    if (isBitSet(_data.batteryStatus, 15)) {
        Serial.println("Warning: Wrong identification for rated voltage!");
    }
    Serial.println("----------------------");
}

void MPPTController::printEquipmentStatus() const {
    Serial.println("--- Equipment Status ---");
    Serial.printf("Overall Status: %s / %s\n", isBitSet(_data.equipmentStatus, 0) ? "Running" : "Standby", isBitSet(_data.equipmentStatus, 1) ? "FAULT" : "Normal");

    // Decode D3-D2 for charging status
    switch ((_data.equipmentStatus >> 2) & 0b11) {
        case 0b00: Serial.println("Charging: None"); break;
        case 0b01: Serial.println("Charging: Float"); break;
        case 0b10: Serial.println("Charging: Boost"); break;
        case 0b11: Serial.println("Charging: Equalization"); break;
    }

    if (isBitSet(_data.equipmentStatus, 4))  Serial.println("Fault: PV Input Short");
    if (isBitSet(_data.equipmentStatus, 10)) Serial.println("Fault: Input Over Current");
    if (isBitSet(_data.equipmentStatus, 13)) Serial.println("Fault: Charging MOSFET Short");
    
    // Decode D15-D14 for input voltage status
    switch ((_data.equipmentStatus >> 14) & 0b11) {
        case 0b00: Serial.println("PV Input: Normal"); break;
        case 0b01: Serial.println("PV Input: No Power Connected"); break;
        case 0b10: Serial.println("PV Input: High Voltage"); break;
        case 0b11: Serial.println("PV Input: Voltage Error"); break;
    }
    Serial.println("------------------------");
}

void mppt_task(void *parameters) {

		// --- Configuration ---
	constexpr gpio_num_t MPPT_RX_PIN = GPIO_NUM_22;
	constexpr gpio_num_t MPPT_TX_PIN = GPIO_NUM_23;
	constexpr int MODBUS_SLAVE_ID = 1;
	const unsigned long PRINT_INTERVAL_MS = 10000; // Print data every 10 seconds

	// Instantiate the controller object, passing the hardware serial port and slave ID.
	MPPTController mppt(Serial2, MODBUS_SLAVE_ID);

	unsigned long lastPrintTime = 0;


    // Start the primary serial for monitoring/debugging
    Serial.begin(115200);

    Serial.println("\n[MPPT] Starting MPPT Controller Sketch...");

    // Start the hardware serial for Modbus communication
    Serial2.begin(115200, SERIAL_8N1, MPPT_RX_PIN, MPPT_TX_PIN);

    // Initialize the controller
    mppt.begin();

    Serial.println("[MPPT] Setup Complete. Entering main loop...");

	while (true) {
		// The update() method handles all Modbus communication and timing internally.
		// Call this as frequently as possible in your loop.
		mppt.update();

		// This block handles printing the data at a fixed interval,
		// completely separate from the data polling schedule.
		if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
			lastPrintTime = millis();

			// Get the latest data from the controller
			const MpptData& data = mppt.getData();

			// Check if data has been successfully updated since the last print
			if (data.lastUpdateTime > 0) {
				Serial.println(); // Add a blank line for readability
				mppt.printRatedData();
				mppt.printBatteryStatus();
				mppt.printEquipmentStatus();
			} else {
				Serial.println("[MPPT] Waiting for first successful data poll...");
			}
		}

		// You can add other non-blocking tasks to your main loop here.
		// A small delay can be added if your loop is otherwise empty.
		vTaskDelay(pdMS_TO_TICKS(100)); // Yield to other tasks, adjust as needed
	}
}


