#pragma once
#include <Arduino.h>

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

struct VoltageData {
    uint16_t cumulative_voltage_decivolts; // Cumulative voltage (0.1V)
    uint16_t current_deciamps; // Current (0.1A)
    uint16_t soc_decipercent; // State of charge (0.1%)
};

struct ChargeDischargeStatus {
    uint8_t state;                    // 0 = stationary, 1 = charging, 2 = discharging
    uint8_t charge_mos;               // 0 = off, 1 = on
    uint8_t discharge_mos;            // 0 = off, 1 = on
    uint8_t bms_life_cycles;          // 0–255
    uint32_t remaining_capacity_raw;  // Units: mAh
};

struct BMSStatus {
    uint8_t num_strings;              // Number of battery strings
    uint8_t num_temp_sensors;         // Number of temperature sensors
    uint8_t charger_status;           // 0 = not connected, 1 = connected
    uint8_t load_status;              // 0 = not connected, 1 = connected
    uint8_t io_bitfield;              // Bitmask of DI/DO pins
};

struct CellVoltageFrame {
    uint8_t frame_index;              // Frame number (0–15), 0xFF = invalid
    uint16_t voltages_mv[3];          // Up to 3 voltages per frame, in millivolts
};

struct TemperatureFrame {
    uint8_t frame_index;              // Frame number (0–2)
    uint8_t raw_temps[7];             // Temperature values with offset of +40°C
};

struct FailureStatus {
    uint8_t raw[8];                   // Full raw 8-byte error status payload
};

struct BMSData {
    VoltageData voltage_data;
    ChargeDischargeStatus charge_discharge_status;
    BMSStatus bms_status;
    CellVoltageFrame cell_voltage_frame[16]; // 16 frames, each with up to 3 voltages
    TemperatureFrame temperature_frame[3];    // 3 frames, each with up to 7 temperatures
    FailureStatus failure_status;              // 8-byte error status
    uint32_t timestamp_ms;
};