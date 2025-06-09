#ifndef MPPTCONTROLLER_H
#define MPPTCONTROLLER_H

#include <Arduino.h>
#include <ModbusMaster.h>

// A clean struct to hold all related data from the MPPT controller.
// Default values are set to indicate that no data has been read yet.
struct MpptData {
    float pvVoltage = -1.0f;
    float pvCurrent = -1.0f;
    float pvPower = -1.0f;
    float batteryVoltage = -1.0f;
    float batteryChargeCurrent = -1.0f;
    float batteryChargePower = -1.0f;
    float batteryOverallCurrent = -1.0f;
    uint16_t batteryStatus = 0;
    uint16_t equipmentStatus = 0;
    unsigned long lastUpdateTime = 0;
};

// This class encapsulates all logic and data related to the MPPT charge controller.
class MPPTController {
public:
    // Constructor: Initializes the controller with a serial interface and Modbus slave ID.
    MPPTController(Stream& serial, uint8_t slaveId);

    // Call once in setup() to configure the ModbusMaster node.
    void begin();

    // Call repeatedly in the main loop to poll for new data.
    // This method handles the timing for polling different registers.
    void update();

    // Safely get a constant reference to the most recent data.
    const MpptData& getData() const { return _data; }

    // Decouples data presentation from acquisition. These can be called anytime.
    void printRatedData() const;
    void printBatteryStatus() const;
    void printEquipmentStatus() const;

private:
    // Using an underscore prefix is a common convention for private members.
    ModbusMaster  _node;
    MpptData      _data;
    Stream&       _serial;
    uint8_t       _slaveId;
    
    // Internal state for polling logic
    uint8_t       _currentRegistryIndex = 0;
    unsigned long _lastPollTime = 0;

    // A list of the private member functions that perform the Modbus reads.
    // This uses a "pointer-to-member-function" type.
    using RegistryPollFunc = bool (MPPTController::*)();
    static const RegistryPollFunc _registryPollFunctions[];

    // Private helper methods that perform the actual Modbus communication.
    // They return true on success and false on failure.
    bool pollRatedData();
    bool pollBatteryCurrent();
    bool pollStatusData();

    // Helper for bitwise operations to make code more readable.
    static bool isBitSet(uint16_t value, uint8_t bit) { return (value >> bit) & 1; }
};

#endif // MPPTCONTROLLER_H
