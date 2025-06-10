#pragma once
#include <Arduino.h>
#include <ModbusMaster.h>
#include "data_mppt.h"

// --- Configuration Constants ---
// The interval to poll the next register in the sequence.
const uint32_t POLL_INTERVAL_MS = 2000;

// A type alias for a function that provides the current timestamp.
// This makes the controller independent of the actual time source.
using TimeProviderFunc = std::function<uint32_t()>;

// This class encapsulates all logic and data related to the MPPT charge controller.
class MPPTController {
public:
    // Constructor: Initializes the controller with a serial interface and Modbus slave ID.
    MPPTController(Stream& serial, uint8_t slaveId, TimeProviderFunc timeProvider);

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
    TimeProviderFunc _timeProvider;
    
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
