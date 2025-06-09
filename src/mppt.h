#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>

// --- Configuration Constants ---
// The interval to poll the next register in the sequence.
const uint32_t POLL_INTERVAL_MS = 2000;

// --- Data Structures for MPPT Data ---

// Holds the core electrical readings as raw integer values.
// These values are scaled by 100 (e.g., cV = centi-volts).
typedef struct {
    uint16_t pv_voltage_cV;
    int16_t  pv_current_cA;
    uint16_t battery_voltage_cV;
    int32_t  battery_current_cA;
} mppt_electrical_data_t;

// Holds the status flag registers.
typedef struct {
    uint16_t battery_status;
    uint16_t charging_equipment_status;
} mppt_state_data_t;

// The main data structure that combines all data with a timestamp.
// This is the object that will be passed through the system (e.g., in queues).
typedef struct {
    mppt_electrical_data_t electrical;
    mppt_state_data_t      state;
    uint32_t               timestamp_ms;
} mppt_data_t;

// This class encapsulates all logic and data related to the MPPT charge controller.
class MPPTController {
public:
    // Constructor: Initializes the controller with a serial interface and Modbus slave ID.
    MPPTController(Stream& serial, uint8_t slaveId);

    // Call once in setup() to configure the ModbusMaster node.
    void begin();

    // Call repeatedly to poll for new data. Handles internal timing.
    void update();

    // Safely get a constant reference to the most recent raw data.
    const mppt_data_t& getData() const { return _data; }

    // "On-demand" conversion methods. They take the raw integer data,
    // convert it to human-readable floats, and print to Serial.
    void printHumanReadableElectricalData() const;
    void printHumanReadableBatteryStatus() const;
    void printHumanReadableEquipmentStatus() const;

private:
    ModbusMaster  _node;
    mppt_data_t   _data; // Uses the new, efficient data structure
    Stream&       _serial;
    uint8_t       _slaveId;
    
    uint8_t       _currentRegistryIndex = 0;
    unsigned long _lastPollTime = 0;

    // A list of the private member functions that perform the Modbus reads.
    using RegistryPollFunc = bool (MPPTController::*)();
    static const RegistryPollFunc _registryPollFunctions[];

    // Private polling methods now return raw data.
    bool pollElectricalData();
    bool pollNetBatteryCurrent();
    bool pollStatusData();

    // Helper for bitwise operations
    static bool isBitSet(uint16_t value, uint8_t bit) { return (value >> bit) & 1; }
};
