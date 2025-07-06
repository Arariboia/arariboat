#include <Arduino.h>
#include "Utilities.hpp"
#include "arariboat/mavlink.h"
#include "MavlinkUtilities.hpp"
#include "esp_console.h" // For console commands

void ProcessMavlinkMessage(mavlink_message_t message) {
  
    // PrintMavlinkMessageInfo(message); //TODO:Reimplement this with the new dialect
}

bool TryParseMavlinkMessage(uint8_t input, mavlink_channel_t channel) {
    mavlink_message_t message;
    mavlink_status_t status;
    if (mavlink_parse_char(channel, input, &message, &status)) {
        ProcessMavlinkMessage(message);
        return true;
    }
    return false;
}

bool TryParseASCIICommand(char input) {
    constexpr int inputBufferLength = 256;
    static char inputBuffer[inputBufferLength];
    static int bufferIndex = 0;

    if (input == '\r') return false;
    if (input == '\n') {
        inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
        if (bufferIndex > 0) { // Only process if command is not empty
            int ret;
            esp_err_t err = esp_console_run(inputBuffer, &ret);
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Command not found: %s\n", inputBuffer);
            } else if (err == ESP_ERR_INVALID_ARG) {
                // The command was found, but the arguments were wrong.
                // The handler function likely printed an error message.
            } else if (err == ESP_OK && ret != ESP_OK) {
                printf("Error: Command returned non zero status: %d\n", ret);
            } else if (err != ESP_OK) {
                printf("Error: %s\n", esp_err_to_name(err));
            }
        }
        bufferIndex = 0; // Reset buffer index for the next command
        return true;   
    }

    inputBuffer[bufferIndex++] = input;
    if (bufferIndex >= inputBufferLength) {
        bufferIndex = 0;
    }
    return false;
}

/// @brief Parses the serial input from the user char-by-char and posts the result to the event loop.
/// The use of the event loop allows the serial parser to be decoupled from the receiver of the parsed data.
void uart_task(void *parameter) {

    while (true) {
        if (!Serial.available()) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        char input = Serial.read();
        TryParseMavlinkMessage(input, MAVLINK_COMM_0);
        TryParseASCIICommand(input);
    }
}


