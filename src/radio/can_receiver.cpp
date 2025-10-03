// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <Arduino.h>
#include "driver/twai.h"
#include "esp_log.h"
#include "motor_can_manager.h"
#include "bms_can_manager.h"
#include "led.hpp"
#include "Utilities.hpp"
#include "queues.hpp"
#include "mavlink_data_conversion.h" // MAVLink data conversion functions

#ifdef DEBUG
// #undef DEBUG // Uncomment to enable debug messages locally in this file
#endif

static const char* TAG = "CAN";

MotorCANManager motor_can_manager;
BMSCANManager bms_can_manager;

#define MAVLINK_STREAM_CAN_ID 0xABC // The single CAN ID for streaming all MAVLink messages


void can_receive_task(void* parameter) {

    /* Setup the queues to integrate with the rest of the system
    They must be initialized before calling these methods */
    bms_can_manager.set_data_queue(broker_queue);
    motor_can_manager.set_data_queue(broker_queue);

    // MAVLink stream reassembly
    mavlink_message_t received_mav_msg;
    mavlink_status_t mav_status = {0}; // Zero-initialize the status struct

    while (true) {
        twai_message_t message;
        if (twai_receive(&message, pdMS_TO_TICKS(portMAX_DELAY)) != ESP_OK) continue;

        if (bms_can_manager.handle_can_frame(message)) continue;
        if (motor_can_manager.handle_can_frame(message)) continue;

        // 1. Check if it's a MAVLink stream frame
        if (message.identifier == MAVLINK_STREAM_CAN_ID) {
            // The mavlink_parse_char function has its own internal state machine for reassembly.
            // We just need to feed it the raw byte stream from our CAN frames.
            for (int i = 0; i < message.data_length_code; i++) {
                // If a complete message is parsed, the function returns true.
                if (mavlink_parse_char(MAVLINK_COMM_0, message.data[i], &received_mav_msg, &mav_status)) {
                    
                    DEBUG_PRINTF("[CAN RX] Parsed MAVLink message ID %d\n", received_mav_msg.msgid);

                    message_t internal_msg;
                    if (message_t_from_mavlink_msg(&received_mav_msg, &internal_msg)) {
                        if (xQueueSend(broker_queue, &internal_msg, pdMS_TO_TICKS(10)) != pdTRUE) {
                            DEBUG_PRINTF("[CAN RX] Failed to queue received message.\n");
                        }
                    } else {
                        DEBUG_PRINTF("[CAN RX] Could not convert MAVLink message ID %d.\n", received_mav_msg.msgid);
                    }
                }
            }
            continue; // Finish processing this CAN frame
        }

        #ifdef DEBUG
        // If no handler processed the message, print it
        char buffer[256] = {0};
        sprintf(buffer, "Received message: ID=0x%X, Length=%d\n", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            sprintf(buffer + strlen(buffer), "0x%02X ", message.data[i]);
        }
        strcat(buffer, "\n");
        Serial.printf("%s", buffer);
        #endif
    }
}

void can_transmit_task(void* parameter) {
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 1 second
        
        // bms_can_manager.poll_bms_data(); // Poll BMS data every second
    }
}

void simple_reception_test() {
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
        char buffer[256] = {0};
        sprintf(buffer, "Received message: ID=0x%X, Length=%d\n", message.identifier, message.data_length_code);
        for (int i = 0; i < message.data_length_code; i++) {
            sprintf(buffer + strlen(buffer), "0x%02X ", message.data[i]);
        }
        strcat(buffer, "\n");
        Serial.print(buffer);
    }
}

void queue_listener_task(void* parameter) {
    QueueHandle_t motor_queue = xQueueCreate(10, sizeof(motor_data_t));
    if (motor_queue == NULL) {
        Serial.println("\nFailed to create motor queue");
        vTaskDelete(NULL);
    }
    motor_can_manager.set_data_queue(motor_queue);

    QueueHandle_t bms_queue = xQueueCreate(10, sizeof(bms_data_t));
    if (bms_queue == NULL) {
        Serial.println("\nFailed to create BMS queue");
        vTaskDelete(NULL);
    }

    bms_can_manager.set_data_queue(bms_queue);

    while (true) {
        motor_data_t motor_data;
        if (xQueueReceive(motor_queue, &motor_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\n[LISTEN]Motor Data: Bus Voltage=%.1fV, Bus Current=%.1fA, Phase Current=%.1fA, RPM=%d\n",
                motor_data.electrical_data.bus_voltage_dV / 10.f, motor_data.electrical_data.bus_current_dA / 10.f, motor_data.electrical_data.phase_current_dA / 10.f, motor_data.electrical_data.rpm);
        }

        bms_data_t bms_data;
        if (xQueueReceive(bms_queue, &bms_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            Serial.printf("\n[LISTEN]BMS Data: Voltage=%.1fV, Current=%.1fA, SOC=%.1f%%\n",
                bms_data.voltage_data.cumulative_voltage_decivolts / 10.f,
                bms_data.voltage_data.current_deciamps / 10.f,
                bms_data.voltage_data.soc_decipercent / 10.f);
        }
    }
    vTaskDelete(NULL); // Delete the task to free up resources
}

void can_task(void* parameter) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_25, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    twai_driver_install(&g_config, &t_config, &f_config);
    twai_start();

    xTaskCreate(can_receive_task, "CANReceiveTask", 4096, NULL, 3, NULL);
    xTaskCreate(can_transmit_task, "CANTransmitTask", 4096, NULL, 2, NULL);
    // xTaskCreate(queue_listener_task, "QueueListenerTask", 4096, NULL, 1, NULL); // Create the queue listener task
    vTaskDelete(NULL); // Delete the bootstrap task to free up resources
}