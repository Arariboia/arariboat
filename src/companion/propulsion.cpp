#include <Arduino.h>
#include "Utilities.hpp"
#include "propulsion.h"
#include "queues.hpp" // Include the queues header to access the system queues

void propulsion_task(void* parameter) {
    Serial.println("[propulsion_task] Starting...");

    // Main loop for the propulsion task
    for (;;) {
        // Wait indefinitely for a message to arrive on the propulsion queue.
        message_t received_message;
        if (xQueueReceive(propulsion_queue, &received_message, portMAX_DELAY) == pdPASS) {
            // Process the received message
            Serial.printf("[propulsion_task] Received message from source: %s\n", DATA_SOURCE_NAMES[received_message.source]);
            
            // Here you can add logic to control the propulsion system based on the received message
            // For example, if the message contains a command to start or stop the motors, handle it accordingly.
        }
    }
}