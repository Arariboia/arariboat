#pragma once
#include "freertos/FreeRTOS.h" // FreeRTOS definitions
#include "freertos/queue.h" // FreeRTOS queue definitions
#include "data.hpp" // data.h contains the interface for the data structures used in the queues

extern QueueHandle_t broker_queue;
extern QueueHandle_t main_radio_queue;
extern QueueHandle_t auxiliary_radio_queue;
extern QueueHandle_t internet_queue;
extern QueueHandle_t logger_queue;
extern QueueHandle_t can_queue; 
extern QueueHandle_t propulsion_queue;

// Function to initialize all queues
extern void initialize_queues();