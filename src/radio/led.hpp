#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/**
 * @brief Defines the available, predefined blink patterns.
 * These are symbolic names for different visual indicators.
 */
typedef enum {
    LED_PATTERN_IDLE,           // LED off, the default state.
    LED_PATTERN_WIFI_CONNECTING,// A slow, steady "heartbeat" blink.
    LED_DUMMY_1,                // A fast, steady blink for searching/activity.
    LED_PATTERN_DATA_XMIT,      // A quick double-blink to indicate data transmission.
    LED_PATTERN_OK,             // A single, solid flash to confirm an action.
    LED_PATTERN_ERROR,          // A continuous, rapid blink to signal a recoverable error.
    LED_PATTERN_CRITICAL_ERROR  // Solid on, indicating a non-recoverable system fault.
} led_pattern_id_t;

/**
 * @brief Defines a command to be sent to the LED Manager task.
 */
typedef struct {
    led_pattern_id_t pattern;   // Which pattern to display.
    uint8_t priority;           // Priority of the pattern (0=lowest, 255=highest). Higher priority patterns override lower ones.
    uint32_t duration_ms;       // How long the pattern should play, in milliseconds. Use 0 for indefinite.
} led_command_t;


// --- Public API ---

/**
 * @brief Initializes the LED manager system. Must be called once in setup().
 * @param led_pin The GPIO pin connected to the LED.
 */
void led_manager_init(uint8_t led_pin);

/**
 * @brief Sends a command to the LED manager to request a blink pattern.
 * This function is thread-safe and can be called from any task.
 * @param command The led_command_t struct describing the request.
 * @return pdTRUE if the command was successfully sent to the queue, pdFALSE otherwise.
 */
BaseType_t led_manager_request_pattern(led_command_t command);

/**
 * @brief The FreeRTOS task function that manages the LED.
 * This should be created with xTaskCreate in your setup function.
 * @param parameter Task parameters (unused).
 */
void led_manager_task(void* parameter);
