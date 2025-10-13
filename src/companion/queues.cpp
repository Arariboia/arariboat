#include "queues.hpp"
#include "freertos/FreeRTOS.h"

QueueHandle_t broker_queue;
QueueHandle_t can_queue; 
QueueHandle_t propulsion_queue;
QueueHandle_t mqtt_queue;

#define message_queue_length 10
#define can_queue_length 10
#define propulsion_queue_length 10
#define mqtt_queue_length 20

// Centralized definition for queue configurations
static const queue_config_t queue_configs[] = {
    { &broker_queue, "Broker Queue", message_queue_length, sizeof(message_t) },
    { &can_queue, "CAN Queue", can_queue_length, sizeof(message_t) },
    { &propulsion_queue, "Propulsion Queue", propulsion_queue_length, sizeof(message_t) },
    { &mqtt_queue, "MQTT Queue", mqtt_queue_length, sizeof(message_t) }
};

#define NUM_QUEUES (sizeof(queue_configs) / sizeof(queue_config_t))

void initialize_queues() {
    for (size_t i = 0; i < NUM_QUEUES; i++) {
        const queue_config_t* config = &queue_configs[i];
        *(config->queue) = xQueueCreate(config->length, config->item_size);
        if (*(config->queue) == NULL) {
            Serial.printf("Failed to create %s\n", config->name);
            //Halt the system
            while (1) {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
    }
}



