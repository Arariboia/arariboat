#include "queues.hpp"
#include "freertos/FreeRTOS.h"

QueueHandle_t broker_queue;
QueueHandle_t main_radio_queue;
QueueHandle_t auxiliary_radio_queue;
QueueHandle_t logger_queue;
QueueHandle_t can_queue; 

#define message_queue_length 10
#define main_radio_queue_length 10
#define auxiliary_radio_queue_length 10
#define logger_queue_length 10
#define can_queue_length 10

// Centralized definition for queue configurations
static const queue_config_t queue_configs[] = {
    { &broker_queue, "Broker Queue", message_queue_length, sizeof(message_t) },
    { &main_radio_queue, "Main Radio Queue", main_radio_queue_length, sizeof(message_t) },
    { &auxiliary_radio_queue, "Auxiliary Radio Queue", auxiliary_radio_queue_length, sizeof(message_t) },
    { &logger_queue, "Logger Queue", logger_queue_length, sizeof(message_t) },
    { &can_queue, "CAN Queue", can_queue_length, sizeof(message_t) }
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
