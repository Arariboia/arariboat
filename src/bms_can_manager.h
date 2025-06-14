#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include <map>
#include <functional>
#include "can_manager.h"
#include "freertos/queue.h" // Add this include


/*
| **Data ID** | **Description**                  | **Direction** | **Bytes**          | **Interpretation**                                                                 |
|-------------|----------------------------------|---------------|--------------------|-------------------------------------------------------------------------------------|
| `0x90`      | Total Voltage, Current, SOC      | BMS → Host    | Byte0–1            | Cumulative voltage (0.1 V)                                                          |
|             |                                  |               | Byte2–3            | Total voltage (0.1 V)                                                               |
|             |                                  |               | Byte4–5            | Current (0.1 A, offset +30000)                                                     |
|             |                                  |               | Byte6–7            | SOC (0.1 %)                                                                         |
| `0x91`      | Max & Min Cell Voltage           | BMS → Host    | Byte0–1            | Max cell voltage (mV)                                                               |
|             |                                  |               | Byte2              | Cell number with max voltage                                                        |
|             |                                  |               | Byte3–4            | Min cell voltage (mV)                                                               |
|             |                                  |               | Byte5              | Cell number with min voltage                                                        |
| `0x92`      | Max & Min Temperature            | BMS → Host    | Byte0              | Max temperature (°C, offset +40)                                                    |
|             |                                  |               | Byte1              | Sensor index with max temperature                                                   |
|             |                                  |               | Byte2              | Min temperature (°C, offset +40)                                                    |
|             |                                  |               | Byte3              | Sensor index with min temperature                                                   |
| `0x93`      | Charge/Discharge & MOS State     | BMS → Host    | Byte0              | System state: 0=idle, 1=charging, 2=discharging                                     |
|             |                                  |               | Byte1              | Charge MOS state                                                                    |
|             |                                  |               | Byte2              | Discharge MOS state                                                                 |
|             |                                  |               | Byte3              | BMS life (cycles)                                                                   |
|             |                                  |               | Byte4–7            | Remaining capacity (mAh)                                                            |
| `0x94`      | BMS Status Information           | BMS → Host    | Byte0              | Battery string count                                                                |
|             |                                  |               | Byte1              | Temperature sensor count                                                            |
|             |                                  |               | Byte2              | Charger status: 0=disconnected, 1=connected                                         |
|             |                                  |               | Byte3              | Load status: 0=disconnected, 1=connected                                            |
|             |                                  |               | Byte4              | Bitfield: DI/DO states (DI1–DI4, DO1–DO4)                                           |
|             |                                  |               | Byte5–7            | Reserved                                                                            |
| `0x95`      | Cell Voltages (1–48)             | BMS → Host    | Byte0              | Frame number (0–15), 0xFF = invalid                                                 |
|             |                                  |               | Byte1–6            | Up to 3 cell voltages (2 bytes each, in mV)                                        |
|             |                                  |               | Byte7              | Reserved                                                                            |
| `0x96`      | Cell Temperatures (1–16)         | BMS → Host    | Byte0              | Frame number (0–2)                                                                  |
|             |                                  |               | Byte1–7            | Up to 7 temperatures (1 byte each, offset +40 °C)                                   |
| `0x97`      | Cell Balancing State             | BMS → Host    | Byte0–7            | Each bit: 0 = closed, 1 = open (up to 64 bits, Cell 1–48 balancing state)          |
| `0x98`      | Battery Fault/Protection Status  | BMS → Host    | Byte0–7            | Bitfields covering voltages, temps, current, MOS, RTC, EEPROM, and other errors    |

*/

typedef struct {
    uint16_t cumulative_voltage_decivolts; // Cumulative voltage (0.1V)
    int16_t current_deciamps; // Current (0.1A)
    uint16_t soc_decipercent; // State of charge (0.1%)
} voltage_data_t;

typedef struct {
    uint8_t state;                    // 0 = stationary, 1 = charging, 2 = discharging
    uint8_t charge_mos;               // 0 = off, 1 = on
    uint8_t discharge_mos;            // 0 = off, 1 = on
    uint8_t bms_life_cycles;          // 0–255
    uint32_t remaining_capacity_raw;  // Units: mAh
} charge_discharge_status_t;

typedef struct {
    uint8_t num_strings;              // Number of battery strings
    uint8_t num_temp_sensors;         // Number of temperature sensors
    uint8_t charger_status;           // 0 = not connected, 1 = connected
    uint8_t load_status;              // 0 = not connected, 1 = connected
    uint8_t io_bitfield;              // Bitmask of DI/DO pins
} bms_status_t;

typedef struct {
    uint8_t frame_index;              // Frame number (0–15), 0xFF = invalid
    uint16_t voltages_mv[3];          // Up to 3 voltages per frame, in millivolts
} cell_voltage_frame_t;

typedef struct {
    uint8_t frame_index;              // Frame number (0–2)
    uint8_t raw_temps[7];             // Temperature values with offset of +40°C
} temperature_frame_t;

typedef struct {
    uint8_t raw[8];                   // Full raw 8-byte error status payload
} failure_status_t;

typedef struct {
    voltage_data_t voltage_data;
    charge_discharge_status_t charge_discharge_status;
    bms_status_t bms_status;
    cell_voltage_frame_t cell_voltage_frame[16]; // 16 frames, each with up to 3 voltages
    temperature_frame_t temperature_frame[3];    // 3 frames, each with up to 7 temperatures
    failure_status_t failure_status;              // 8-byte error status
} bms_data_t;


class BMSCANManager : public ICANManager {
public:
    BMSCANManager();
    bool handle_can_frame(const twai_message_t& message) override;
    void poll_bms_data(); // Call this periodically to process BMS 
    void print_can_handlers();
    void set_data_queue(QueueHandle_t queue); // Add this method

private:

    bms_data_t _bms_data;

    using CANHandler = std::function<void(const twai_message_t&)>;
    std::map<uint32_t, CANHandler> _can_handlers;
    void initialize_can_handlers();

    //Response handlers
    void handle_voltage_response(const twai_message_t& message);
    void handle_charge_discharge_response(const twai_message_t& message);
    void handle_status_response(const twai_message_t& message);
    void handle_cell_voltage_response(const twai_message_t& message);
    void handle_temperature_response(const twai_message_t& message);
    void handle_failure_response(const twai_message_t& message);
    void send_poll_command(uint32_t data_id);

    // Add these:
    QueueHandle_t _data_queue = nullptr;
    uint8_t _received_flags = 0;
    static constexpr uint8_t FLAG_VOLTAGE = 1 << 0;
    static constexpr uint8_t FLAG_CHARGE_DISCHARGE = 1 << 1;
    static constexpr uint8_t FLAG_STATUS = 1 << 2;
    static constexpr uint8_t FLAG_CELL_VOLTAGE = 1 << 3;
    static constexpr uint8_t FLAG_TEMPERATURE = 1 << 4;
    static constexpr uint8_t FLAG_FAILURE = 1 << 5;
    static constexpr uint8_t ALL_FLAGS = FLAG_VOLTAGE | FLAG_CHARGE_DISCHARGE | FLAG_STATUS | FLAG_CELL_VOLTAGE | FLAG_TEMPERATURE | FLAG_FAILURE;
    void check_and_publish();
};
