#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include <map>
#include <functional>
#include "can_manager.h"
#include "freertos/queue.h" // Add this include
#include "data_bms.h"

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
