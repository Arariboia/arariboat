#pragma once
#include "event_loop.hpp" //Event loop to handle events between tasks. This allows tasks to communicate with each other with loosely coupled code.
#include "led.hpp" // LED class for controlling the onboard LED.

//*********************************************************/
#define DEBUG // Uncomment to enable debug messages globally/
//*********************************************************/

#ifdef DEBUG
#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#define DEBUG_PRINT(...)
#endif

//Preprocessor trick to convert a macro to a string
#define STRINGIFY(x) __STRINGIFY__(x)
#define __STRINGIFY__(x) #x

//Macro to compare two strings
#define STRINGS_ARE_EQUAL(a, b) strcmp(a, b) == 0


//Macro to synchronize the creation of tasks with their names and their handles
//Trivial macros to improve argument readability

#define CREATE_TASK_WITH_HANDLE(task_name, stack_size, priority) xTaskCreate(task_name, #task_name, stack_size, NULL, priority, &task_name##Handle)
#define CREATE_TASK(task_name, stack_size, priority) xTaskCreate(task_name, #task_name, stack_size, NULL, priority, NULL)

#define STACK_SIZE(x) (x) 
#define PRIORITY(x) (x)


void led_manager_task(void* parameter);
void WifiTask(void* parameter);
void ServerTask(void* parameter);
void SerialTask(void* parameter);
void TemperatureTask(void* parameter);
void GPSTask(void* parameter);
void InstrumentationTask(void* parameter);
void time_manager_task(void* parameter);
void mppt_task(void *parameters);
void broker_task(void* parameter);
void led_manager_task(void* parameter);
void can_task(void* parameter);


// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

extern TaskHandle_t LedBlinkerTaskHandle;
extern TaskHandle_t WifiTaskHandle;
extern TaskHandle_t ServerTaskHandle;
extern TaskHandle_t SerialTaskHandle;
extern TaskHandle_t TemperatureTaskHandle;
extern TaskHandle_t GPSTaskHandle;
extern TaskHandle_t InstrumentationTaskHandle;
extern TaskHandle_t time_manager_task_handle;

/// @brief Calibrates a reading by using a linear equation obtained by comparing the readings with a multimeter.
/// @return Calibrated reading
inline float LinearCorrection(const float input_value, const float slope, const float intercept) {
    return slope * input_value + intercept;
}

