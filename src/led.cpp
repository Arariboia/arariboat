#include "led.hpp"
#include "Utilities.hpp" // For DEBUG_PRINTF and other utility macros
#include "esp_console.h"
#include "argtable3/argtable3.h"

// #undef DEBUG // Uncomment to enable debug messages locally in this file

// --- Private (Static) Globals ---
static QueueHandle_t s_led_command_queue = NULL;
static uint8_t s_led_pin = -1; // Use an invalid pin number to indicate "not initialized"

// --- Pattern Definitions ---
// Each pattern is defined by {on_time_ms, off_time_ms}
static const uint16_t s_patterns[][2] = {
    [LED_PATTERN_IDLE]            = {0, 1000},    // Solid off (delay handled by off_time)
    [LED_PATTERN_WIFI_CONNECTING] = {500, 500},   // Slow blink
    [LED_DUMMY_1]                 = {150, 150},   // Fast blink
    [LED_PATTERN_DATA_XMIT]       = {50, 50},   // Note: The double-blink is special-cased in the task
    [LED_PATTERN_OK]              = {500, 0},     // Solid ON for the duration, then reverts to IDLE
    [LED_PATTERN_ERROR]           = {100, 100},   // Rapid blink
    [LED_PATTERN_CRITICAL_ERROR]  = {1000, 0}     // Solid ON, indefinitely
};


void led_manager_init(uint8_t led_pin) {
    s_led_pin = led_pin;
    pinMode(s_led_pin, OUTPUT);
    digitalWrite(s_led_pin, LOW); // Start with LED off
    // Create a queue that can hold up a given number of led_command_t structures.
    s_led_command_queue = xQueueCreate(20, sizeof(led_command_t));
}

BaseType_t led_manager_request_pattern(led_command_t command) {
    if (s_led_command_queue == NULL) {
        return pdFALSE;
    }
    // Send the command to the queue. Don't wait if the queue is full.
    return xQueueSend(s_led_command_queue, &command, 0);
}

void register_led_commands() {
    //Register the LED commands with the ESP console.
    const esp_console_cmd_t led_cmds[] = {
        {
            .command = "led",
            .help = "Control the LED patterns.",
            .hint = NULL,
            .func = [](int argc, char** argv) -> int {
                if (argc < 2) {
                    Serial.println("Usage: led <pattern> [duration_ms] [priority]");
                    return 0;
                }
                led_pattern_id_t pattern = static_cast<led_pattern_id_t>(atoi(argv[1]));
                uint32_t duration_ms = (argc > 2) ? static_cast<uint32_t>(atoi(argv[2])) : 0;
                uint8_t priority = (argc > 3) ? static_cast<uint8_t>(atoi(argv[3])) : 0;

                led_command_t command = {pattern, priority, duration_ms};
                if (led_manager_request_pattern(command) == pdTRUE) {
                    Serial.printf("LED command queued: Pattern %d, Priority %d, Duration %d ms\n", pattern, priority, duration_ms);
                } else {
                    Serial.println("Failed to queue LED command.");
                }
                return 0;
            },
            .argtable = NULL
        }
    };

    for (const auto& cmd : led_cmds) {
        esp_err_t err = esp_console_cmd_register(&cmd);
        if (err != ESP_OK) {
            Serial.printf("[LedManager] Error registering command '%s': %s\n", cmd.command, esp_err_to_name(err));
        }
    }
}

void led_manager_task(void* parameter) {

    led_manager_init(GPIO_NUM_2); // Initialize the LED pin and command queue

    // Basic check to ensure the system was initialized before the task started.
    if (s_led_pin == (uint8_t)-1 || s_led_command_queue == NULL) {
        Serial.println("[LedManager] Error: Not initialized. Deleting task.");
        vTaskDelete(NULL);
    }
    Serial.println("[LedManager] Task started.");

    // The currently active command. Start with IDLE.
    led_command_t current_command = {
        .pattern = LED_PATTERN_IDLE,
        .priority = 0,
        .duration_ms = 0 // Indefinite
    };
    uint64_t pattern_start_time_ms = 0;

    for (;;) {
        led_command_t new_command;
        // Check for a new command, but don't block. The task must keep running to execute the current pattern.
        if (xQueueReceive(s_led_command_queue, &new_command, 0) == pdPASS) {
            // A new command has arrived. If its priority is high enough, it becomes the new active pattern.
            if (new_command.priority >= current_command.priority) {
                current_command = new_command;
                pattern_start_time_ms = millis(); // Reset the duration timer for the new pattern.
                DEBUG_PRINTF("[LedManager] New pattern started: %d, Priority: %d\n", current_command.pattern, current_command.priority);
            }
        }

        // Check if the current pattern has a limited duration and if that duration has expired.
        if (current_command.duration_ms > 0 && (millis() - pattern_start_time_ms > current_command.duration_ms)) {
            // Pattern expired, revert to the default IDLE state.
            current_command = {.pattern = LED_PATTERN_IDLE, .priority = 0, .duration_ms = 0};

            //Check the size of queue and if empty set to idle state
            UBaseType_t queue_size = uxQueueMessagesWaiting(s_led_command_queue);
            if (queue_size == 0 && current_command.pattern != LED_PATTERN_IDLE) {
                // If the queue is empty and we're not in IDLE, revert to IDLE.
                current_command = {.pattern = LED_PATTERN_IDLE, .priority = 0, .duration_ms = 0};
                DEBUG_PRINTF("[LedManager] Queue empty, reverting to IDLE state.\n");
        }
        }

        // --- Execute the current pattern's logic ---
        const uint16_t on_time = s_patterns[current_command.pattern][0];
        const uint16_t off_time = s_patterns[current_command.pattern][1];

        if (on_time > 0) {
            digitalWrite(s_led_pin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(on_time));
        }

        // Special case for the double-blink pattern.
        if (current_command.pattern == LED_PATTERN_DATA_XMIT) {
            digitalWrite(s_led_pin, LOW);
            vTaskDelay(pdMS_TO_TICKS(100)); // Brief pause
            digitalWrite(s_led_pin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(on_time));
        }

        if (off_time > 0) {
            digitalWrite(s_led_pin, LOW);
            vTaskDelay(pdMS_TO_TICKS(off_time));
        }
    }
}
