#include "motor_can_manager.h"
#include <Arduino.h>
#include "esp_log.h"
#include "data.hpp"
#include "time_manager.h"

#define CAN_ELECTRICAL_DATA_ID_LEFT  0x180117F0
#define CAN_STATE_DATA_ID_LEFT 0x180217F0
#define CAN_ELECTRICAL_DATA_ID_RIGHT  0x180117EF
#define CAN_STATE_DATA_ID_RIGHT 0x180217EF

static const char* TAG = "CAN";

MotorCANManager::MotorCANManager() {
    initialize_can_handlers();
}

bool MotorCANManager::handle_can_frame(const twai_message_t& message) {
    if (can_handlers.find(message.identifier) != can_handlers.end()) {
        can_handlers[message.identifier](message);
        return true;
    }
    return false;
}

void MotorCANManager::handle_electrical_data(const twai_message_t& message, motor_electrical_data_t& data) {
    data.bus_voltage_dV = (message.data[0] | (message.data[1] << 8));
    data.bus_current_dA = (message.data[2] | (message.data[3] << 8)) - 32000;
    data.phase_current_dA = (message.data[4] | (message.data[5] << 8)) - 32000;
    data.rpm = (message.data[6] | (message.data[7] << 8)) - 32000;
    
    if (message.identifier == CAN_ELECTRICAL_DATA_ID_LEFT) {
        _received_flags_left |= FLAG_ELECTRICAL;
    } else if (message.identifier == CAN_ELECTRICAL_DATA_ID_RIGHT) {
        _received_flags_right |= FLAG_ELECTRICAL;
    }
    check_and_publish(); // Check and publish data if all flags are set
    
    Serial.printf("Received Electrical Data: Bus Voltage=%.1fV, Bus Current=%.1fA, Phase Current=%.1fA, RPM=%d\n",
        data.bus_voltage_dV / 10.f, data.bus_current_dA / 10.f, data.phase_current_dA / 10.f, data.rpm);

}

void MotorCANManager::handle_state_data(const twai_message_t& message, motor_state_data_t& data) {
    data.controller_temp_C = message.data[0] - 40;
    data.motor_temp_C = message.data[1] - 40;
    data.accelerator_percent = message.data[2];
    data.status = message.data[3];
    data.error = message.data[4] | (message.data[5] << 8) | (message.data[6] << 16) | (message.data[7] << 24);

    if (message.identifier == CAN_STATE_DATA_ID_LEFT) {
        _received_flags_left |= FLAG_STATE;
    } else if (message.identifier == CAN_STATE_DATA_ID_RIGHT) {
        _received_flags_right |= FLAG_STATE;
    }
    check_and_publish(); // Check and publish data if all flags are set
    
    Serial.printf("Received State Data: Controller Temp=%dC, Motor Temp=%dC, Accelerator=%d%%, Status=0x%02X, Error=0x%02X\n",
        data.controller_temp_C, data.motor_temp_C, data.accelerator_percent, data.status, data.error);
    decode_motor_status(data.status);
    decode_motor_error(data.error);
}

void MotorCANManager::set_data_queue(QueueHandle_t queue) {
    _data_queue = queue;
}

void MotorCANManager::check_and_publish() {
    if (_received_flags_left == (FLAG_ELECTRICAL | FLAG_STATE) && _data_queue) {
        message_t msg;
        msg.source = DATA_SOURCE_MOTOR_LEFT;
        msg.payload.motor = motor_data_left;
        msg.timestamp.time_since_boot_ms = millis();
        msg.timestamp.epoch_seconds = get_epoch_seconds();
        msg.timestamp.epoch_ms = get_epoch_millis();

        xQueueSend(_data_queue, &msg, 0);
        _received_flags_left = 0; // Reset for next cycle
    }

    if (_received_flags_right == (FLAG_ELECTRICAL | FLAG_STATE) && _data_queue) {
        message_t msg;
        msg.source = DATA_SOURCE_MOTOR_RIGHT;
        msg.payload.motor = motor_data_right;
        msg.timestamp.time_since_boot_ms = millis();
        msg.timestamp.epoch_seconds = get_epoch_seconds();
        msg.timestamp.epoch_ms = get_epoch_millis();
        xQueueSend(_data_queue, &msg, 0);
        _received_flags_right = 0; // Reset for next cycle
    }
}

void MotorCANManager::initialize_can_handlers() {
    can_handlers[CAN_ELECTRICAL_DATA_ID_LEFT] = [this](const twai_message_t& message) {
        handle_electrical_data(message, motor_data_left.electrical_data);
    };
    can_handlers[CAN_STATE_DATA_ID_LEFT] = [this](const twai_message_t& message) {
        handle_state_data(message, motor_data_left.state_data);
    };
    can_handlers[CAN_ELECTRICAL_DATA_ID_RIGHT] = [this](const twai_message_t& message) {
        handle_electrical_data(message, motor_data_right.electrical_data);
    };
    can_handlers[CAN_STATE_DATA_ID_RIGHT] = [this](const twai_message_t& message) {
        handle_state_data(message, motor_data_right.state_data);
    };
}

void MotorCANManager::decode_motor_status(uint8_t status) {
    uint8_t gear = status & (BIT2 | BIT1 | BIT0); // Mask for gear bits
    const char* gear_str = "";
    switch (gear) {
        case 0b000: gear_str = "NO"; break;
        case 0b001: gear_str = "R"; break;
        case 0b010: gear_str = "N"; break;
        case 0b011: gear_str = "D1"; break;
        case 0b100: gear_str = "D2"; break;
        case 0b101: gear_str = "D3"; break;
        case 0b110: gear_str = "S"; break;
        case 0b111: gear_str = "P"; break;
        default: gear_str = "Unknown gear"; break;
    }

    bool brake = (status & BIT3) >> 3; // Brake bit
    const char* brake_str = brake ? "ON" : "OFF";

    uint8_t operation_mode = (status & (BIT6 | BIT5 | BIT4)) >> 4;
    const char* operation_mode_str = "";
    switch (operation_mode) {
        case 0b000: operation_mode_str = "Stopped"; break;
        case 0b001: operation_mode_str = "Drive"; break;
        case 0b010: operation_mode_str = "Cruise"; break;
        case 0b011: operation_mode_str = "EBS"; break;
        case 0b100: operation_mode_str = "Hold"; break;
    }

    uint8_t dc_contactor = (status & BIT7) >> 7; // DC contactor bit
    const char* dc_contactor_str = dc_contactor ? "ON" : "OFF";

    printf("Gear: %s, Brake: %s, Operation Mode: %s, DC Contactor: %s\n",
        gear_str, brake_str, operation_mode_str, dc_contactor_str);
}

void MotorCANManager::decode_motor_error(uint32_t error) {
    std::map<uint32_t, const char*> error_map = {
        {BIT0, "Motor Overcurrent"},
        {BIT1, "Motor Overload"},
        {BIT2, "Motor Overvoltage"},
        {BIT3, "Motor Undervoltage"},
        {BIT4, "Controller Overheat"},
        {BIT5, "Motor Overheat"},
        {BIT6, "Motor Stalled"},
        {BIT7, "Motor Out of Phase"},
        {BIT8, "Motor Sensor Failure"},
        {BIT9, "Motor AUX Sensor"},
        {BIT10, "Encoder Misaligned"},
        {BIT11, "Anti-Runaway Engaged"},
        {BIT12, "Main Accelerator"},
        {BIT13, "AUX Accelerator"},
        {BIT14, "Pre-charge"},
        {BIT15, "DC Contactor"},
        {BIT16, "Power Valve"},
        {BIT17, "Current Sensor"},
        {BIT18, "Auto-tune"},
        {BIT19, "RS485"},
        {BIT20, "CAN"},
        {BIT21, "Software"}
    };

    for (const auto& pair : error_map) {
        if (error & pair.first) {
            Serial.printf("Error: %s\n", pair.second);
        }
    }
}
