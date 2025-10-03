#pragma once
#include <Arduino.h>
#include "driver/twai.h"
#include <map>
#include <functional>
#include "can_manager.h"
#include "data_motor.h"

class MotorCANManager : public ICANManager {
public:

    MotorCANManager();
    bool handle_can_frame(const twai_message_t& message) override;
    void set_data_queue(QueueHandle_t queue);

private:
    motor_data_t motor_data_left;
    motor_data_t motor_data_right;
    using CANHandler = std::function<void(const twai_message_t&)>;
    std::map<uint32_t, CANHandler> can_handlers;
    void initialize_can_handlers();
    void handle_electrical_data(const twai_message_t& message, motor_electrical_data_t& data);
    void handle_state_data(const twai_message_t& message, motor_state_data_t& data);
    void decode_motor_status(uint8_t status);
    void decode_motor_error(uint32_t error);

    QueueHandle_t _data_queue = nullptr;
    uint8_t _received_flags_left = 0;
    uint8_t _received_flags_right = 0;
    static constexpr uint8_t FLAG_ELECTRICAL = 1 << 0;
    static constexpr uint8_t FLAG_STATE = 1 << 1;
    static constexpr uint8_t ALL_FLAGS = FLAG_ELECTRICAL | FLAG_STATE;
    void check_and_publish();
};
