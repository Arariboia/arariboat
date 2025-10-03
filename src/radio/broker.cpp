#include "broker.h"
#include "data.hpp"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "radio_defs.h"

void broker_task(void* parameter) {
    Serial.print("[broker_task] Starting... ");
    message_t received_message;

    for (;;) {
        // Wait indefinitely for a message to arrive on the central broker queue.
        // The task will block here, consuming no CPU, until a message is available.
        if (xQueueReceive(broker_queue, &received_message, portMAX_DELAY) == pdPASS) {
            
            // --- Delegation Logic ---
            // Here, you can add rules about where to send messages.
            
            Serial.printf("[broker_task] Received message from source: %s\n", DATA_SOURCE_NAMES[received_message.source]);
            
            // // 1. Send to the Logger Task
            // if (logger_queue != NULL) {
            //     // Send a copy of the message. Use a small timeout to prevent the broker
            //     // from blocking if the logger's queue is full.
            //     if (xQueueSend(logger_queue, &received_message, pdMS_TO_TICKS(10)) != pdPASS) {
            //         Serial.println("[broker_task] Warning: Failed to send message to Logger queue.");
            //     }
            // }

            // // 2. Send to the Radio Task
            // if (main_radio_queue != NULL) {
            //     // Send a copy of the same message to the radio task.
            //     if (xQueueSend(main_radio_queue, &received_message, pdMS_TO_TICKS(10)) != pdPASS) {
            //         Serial.println("[broker_task] Warning: Failed to send message to primary radio queue.");
            //     }
            // }

            // 3. Send to the Auxiliary Radio Task (if applicable)
            #ifdef TRANSMITTER
            if (auxiliary_radio_queue != NULL) {
                // Send a copy of the same message to the auxiliary radio task.
                if (xQueueSend(auxiliary_radio_queue, &received_message, pdMS_TO_TICKS(10)) != pdPASS) {
                    Serial.println("[broker_task] Warning: Failed to send message to auxiliary radio queue.");
                }
            }
            #endif

            // You could add more complex logic here, for example:
            // if (received_message.source == DATA_SOURCE_GPS) {
            //     // Only send GPS data to the radio, not other types.
            //     xQueueSend(xQueueRadio, &received_message, pdMS_TO_TICKS(10));
            // }
        }
    }
}
