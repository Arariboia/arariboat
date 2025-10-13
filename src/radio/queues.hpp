#pragma once
#include "freertos/FreeRTOS.h" // FreeRTOS definitions
#include "freertos/queue.h" // FreeRTOS queue definitions
#include "data.hpp" // data.h contains the interface for the data structures used in the queues

extern QueueHandle_t broker_queue;
extern QueueHandle_t main_radio_queue;
extern QueueHandle_t auxiliary_radio_queue;
extern QueueHandle_t logger_queue;
extern QueueHandle_t can_queue; 

// Structure to define queue properties
typedef struct {
    QueueHandle_t* queue; // Pointer to the queue handle
    const char* name; // Name of the queue for identification
    UBaseType_t length; // Length of the queue
    UBaseType_t item_size; // Size of each item in the queue
} queue_config_t;

// Function to initialize all queues
extern void initialize_queues();