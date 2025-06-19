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

/**
 * @brief Converts a buffer of bytes into a hexadecimal string representation (Pure C).
 *
 * This utility requires the caller to provide a destination buffer. It will
 * write the resulting null-terminated hex string into that buffer.
 *
 * @param dest A pointer to the character buffer where the hex string will be stored.
 * @param dest_size The total size of the destination buffer. Must be at least (src_len * 2 + 1).
 * @param src A pointer to the byte buffer to convert.
 * @param src_len The number of bytes in the source buffer to convert.
 * @return `true` on success, `false` on failure (e.g., destination buffer is too small).
 */
static inline bool bytes_to_hex_string(char* dest, size_t dest_size, const uint8_t* src, size_t src_len) {
    // Check for invalid arguments
    if (dest == NULL || src == NULL) {
        return false;
    }

    // Check if the destination buffer is large enough to hold the hex string
    // We need 2 characters for each source byte, plus 1 for the null terminator.
    if (dest_size < (src_len * 2 + 1)) {
        return false;
    }

    char* dest_ptr = dest;
    for (size_t i = 0; i < src_len; ++i) {
        // Format the byte as a two-digit, uppercase hexadecimal number.
        // `sprintf` could be used, but `snprintf` is safer as it prevents buffer overflows,
        // even though we've already checked the total size.
        int written = snprintf(dest_ptr, 3, "%02X", src[i]);
        
        // Move the pointer forward by the number of characters written (should always be 2)
        dest_ptr += written;
    }

    // Null-terminate the destination string.
    *dest_ptr = '\0';

    return true;
}

// --- Example Usage ---
// This demonstrates how you would use the C version of the function.
// You do not need to copy this part, just the function above.

// // Assume Serial.printf exists (as in Arduino/ESP-IDF)
// // If in pure C, you would use standard printf from <stdio.h>
// void mavlink_debug_example_c() {
//     // A sample MAVLink message buffer
//     uint8_t mav_buffer[] = {0xFE, 0x09, 0x55, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x05, 0x03, 0x51, 0x04};
//     uint16_t mav_len = sizeof(mav_buffer);

//     // Create a destination buffer on the stack.
//     // Its size MUST be at least (mav_len * 2 + 1).
//     char hex_output_buffer[sizeof(mav_buffer) * 2 + 1];

//     // Call the conversion function
//     if (bytes_to_hex_string(hex_output_buffer, sizeof(hex_output_buffer), mav_buffer, mav_len)) {
//         // Print the result. No .c_str() is needed as it's already a char array.
//         printf("[MAV][DEBUG] Packet = %s\n", hex_output_buffer);
//     } else {
//         printf("[MAV][ERROR] Failed to convert packet to hex string.\n");
//     }
// }


void led_manager_task(void* parameter);
void wifi_task(void* parameter);
void server_task(void* parameter);
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

