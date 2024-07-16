#include <Arduino.h>
#include "Utilities.hpp"

#define STRINGS_ARE_EQUAL(a, b) strcmp(a, b) == 0

static void commandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data; 

    if (STRINGS_ARE_EQUAL(command, "blinkfast")) {
        xTaskNotify(LedBlinkerTaskHandle, BlinkRate::Fast, eSetValueWithOverwrite);
    } else if (STRINGS_ARE_EQUAL(command, "blinkmedium")) {
        xTaskNotify(LedBlinkerTaskHandle, BlinkRate::Medium, eSetValueWithOverwrite);
    } else if (STRINGS_ARE_EQUAL(command, "blinkslow")) {
        xTaskNotify(LedBlinkerTaskHandle, BlinkRate::Slow, eSetValueWithOverwrite);
    } else if (STRINGS_ARE_EQUAL(command, "blink")) {
        xTaskNotify(LedBlinkerTaskHandle, BlinkRate::Pulse, eSetValueWithOverwrite);
    }   
}

static void FastBlinkPulse(int pin) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void BlinkNotify(const char* command) {
    char commandCopy[32];
    strncpy(commandCopy, command, 32);

    esp_event_post_to(eventLoop, COMMAND_BASE, 0, commandCopy, strlen(commandCopy) + 1, portMAX_DELAY);
} 


static void BuzzerWrite(int blink_rate) {
    constexpr uint8_t pinBuzzer = PIN_BUZZER;
    static uint8_t pattern_position = 0;

    //ledcAttachPin(pinBuzzer, 0);
    ledcSetup(0, 5000, 8); // 5 kHz PWM, 8-bit resolution
    ledcAttachPin(pinBuzzer, 0);

    constexpr uint8_t pattern[] = {1, 0, 1, 0, 1, 1, 0, 0}; // Example pattern

    // Calculate the position within the pattern
    constexpr uint8_t patternSize = sizeof(pattern) / sizeof(pattern[0]);
    uint8_t patternPosition = pattern_position % patternSize;

    // Determine the buzzer state based on the pattern
    bool buzzerState = pattern[patternPosition] != 0;

    if (blink_rate == BlinkRate::Fast) {
        ledcWrite(0, buzzerState ? 120 : 0);
    } else {
        ledcWrite(0, 0);
    }

    pattern_position++;
}

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void LedBlinkerTask(void* parameter) {

    constexpr uint8_t pinLED = PIN_LED;
    pinMode(pinLED, OUTPUT);
    uint32_t blink_rate = BlinkRate::Slow;
    uint32_t previous_blink_rate = blink_rate;

    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, COMMAND_BASE, ESP_EVENT_ANY_ID, commandCallback, nullptr);

    while (true) {

        static uint32_t previous_blink_time = millis();
        if (millis() - previous_blink_time > blink_rate) {
            previous_blink_time = millis();
            BuzzerWrite(blink_rate);
            digitalWrite(pinLED, !digitalRead(pinLED));
        }
           
        // Set blink rate to the value received from the notification
        static uint32_t received_value = BlinkRate::Slow;
        if (xTaskNotifyWait(0, 0, (uint32_t*)&received_value, 100)) {
            if (received_value == BlinkRate::Pulse) {
                FastBlinkPulse(pinLED);
            } else {
                blink_rate = received_value;
            }
        }
    }
}