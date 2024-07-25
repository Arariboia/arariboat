#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Custom utility macros and functions.

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Implement auxiliary battery and pumps readings using some I2C system
//TODO: Assign better weights to task priorities via benchmarks

//Commands that can be received from serial port or sent from other tasks
ESP_EVENT_DEFINE_BASE(COMMAND_BASE);

/*
Declare task handles for each task to allow its manipulation from other 
tasks, such as sending notifications, resuming or suspending.
Their names must be the same as the task function name followed by "Handle".
*/

TaskHandle_t LedBlinkerTaskHandle = nullptr;
TaskHandle_t WifiTaskHandle = nullptr;
TaskHandle_t ServerTaskHandle = nullptr;
TaskHandle_t SerialTaskHandle = nullptr;
TaskHandle_t TemperatureTaskHandle = nullptr;
TaskHandle_t GPSTaskHandle = nullptr;
TaskHandle_t InstrumentationTaskHandle = nullptr;
TaskHandle_t TimestampTaskHandle = nullptr;
TaskHandle_t FrequencyCounterTaskHandle = nullptr;

void setup() {

    Serial.begin(BAUD_RATE);
    InitializeEventLoop(&eventLoop); // Initialize the event loop to handle events between tasks.
    CREATE_TASK(LedBlinkerTask, STACK_SIZE(2048), PRIORITY(1));
    CREATE_TASK(SerialTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(WifiTask, STACK_SIZE(4096), PRIORITY(2));
    CREATE_TASK(ServerTask, STACK_SIZE(8096), PRIORITY(3));
    CREATE_TASK(TimestampTask, STACK_SIZE(4096), PRIORITY(2));
    CREATE_TASK(TemperatureTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(GPSTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(InstrumentationTask, STACK_SIZE(4096), PRIORITY(3));
    //CREATE_TASK(FrequencyCounterTask, STACK_SIZE(4096), PRIORITY(1));
}

void loop() {
    SystemData::getInstance().WriteMavlinkData();
    vTaskDelay(1000);
}



