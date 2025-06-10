#include <Arduino.h>
#include "Utilities.hpp"

static void FastBlinkPulse(int pin) {
    for (int i = 0; i < 4; i++) {
        digitalWrite(pin, HIGH); vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(pin, LOW);  vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void LedBlinkerTask(void* parameter) {

    constexpr uint8_t pinLED = PIN_LED;
    pinMode(pinLED, OUTPUT);
    uint32_t blink_rate = BlinkRate::Slow;
    uint32_t previous_blink_rate = blink_rate;

    while (true) {

        static uint32_t previous_blink_time = millis();
        if (millis() - previous_blink_time > blink_rate) {
            previous_blink_time = millis();
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