#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "queues.hpp" // Header file for queue initialization and definitions.
#include "ESP32Time.h" // Internal RTC timer of ESP32 to keep track of time
#include "time_manager.h" // Header file for time management tasks.

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Implement auxiliary battery and pumps readings using some I2C system  (DONE)
//TODO: Assign better weights to task priorities via benchmarks

//Commands that can be received from serial port or sent from other tasks
ESP_EVENT_DEFINE_BASE(COMMAND_BASE);

// --- Global Objects (Single source of truth) ---
ESP32Time RTC;
EventGroupHandle_t system_event_group;


void setup() {
    Serial.begin(BAUD_RATE);
    initialize_event_loop(&eventLoop); // Initialize the event loop to handle events between tasks.
    initialize_queues(); // Initialize the queues used for inter-task communication.
    system_event_group = xEventGroupCreate(); // Create an event group for system-wide events.
    CREATE_TASK(LedBlinkerTask, STACK_SIZE(2048), PRIORITY(1));
    // CREATE_TASK(SerialTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(WifiTask, STACK_SIZE(4096), PRIORITY(2));
    // CREATE_TASK(ServerTask, STACK_SIZE(8096), PRIORITY(3));
    CREATE_TASK(time_manager_task, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(TemperatureTask, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(GPSTask, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(InstrumentationTask, STACK_SIZE(4096), PRIORITY(3));
    CREATE_TASK(mppt_task, STACK_SIZE(4096), PRIORITY(3));
    CREATE_TASK(broker_task, STACK_SIZE(4096), PRIORITY(2));
}

void loop() {
    vTaskDelete(NULL); // Delete the loop task, as we are using FreeRTOS tasks instead of the Arduino loop.
}



