#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Utility functions for the project
#include "event_loop.hpp" // Event loop for handling events between tasks.
#include "ESP32Time.h" // Internal RTC timer of ESP32 to keep track of time
#include "queues.hpp" // Header file for queue initialization and definitions.
#include "radio_defs.h" 

//TODO: Move I2C instrumentation here to read from the ADS1115 ADC to fix I2C screen conflict
//TODO: Use IDF event loops for serial communication and intertask communication
//TODO: Adaptative bandwidth and spreading factor based on packet loss and distance


// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerHandle = nullptr;
TaskHandle_t wifiTaskHandle = nullptr;
TaskHandle_t serverHandle = nullptr;
TaskHandle_t serialReaderHandle = nullptr;

// --- Global Objects (Single source of truth) ---
ESP32Time RTC;
EventGroupHandle_t system_event_group;

void initialize_flash_memory();
void wifi_task(void* parameter);
void server_task(void* parameter);
void serial_reader_task(void* parameter);
void radio_task(void* parameter);
void flashcard_reader_task(void *parameter);
void broker_task(void* parameter);
void display_screen_task(void* parameter);

void setup() {
    
    Serial.begin(BAUD_RATE);

    initialize_event_loop(&eventLoop);
    initialize_queues(); // Initialize the queues used for inter-task communication.
    system_event_group = xEventGroupCreate(); // Create an event group for system-wide events.
    CREATE_TASK(broker_task, STACK_SIZE(16384), PRIORITY(2));
    // CREATE_TASK(serial_reader_task, STACK_SIZE(8192), PRIORITY(1));
    // CREATE_TASK(wifi_task, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(server_task, STACK_SIZE(4096), PRIORITY(1));
    // initialize_flash_memory();
    #ifdef TRANSMITTER
    CREATE_TASK(can_task, STACK_SIZE(8192), PRIORITY(3));
    #endif
    // CREATE_TASK(flashcard_reader_task, STACK_SIZE(8192), PRIORITY(1));
    CREATE_TASK(radio_task, STACK_SIZE(8192), PRIORITY(3));
    CREATE_TASK(led_manager_task, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(display_screen_task, STACK_SIZE(4096), PRIORITY(1));
}

void loop() {
    vTaskDelete(NULL);
}


