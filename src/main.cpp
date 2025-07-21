#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <TFT_eSPI.h>     // Hardware-specific library
#include <TFT_eWidget.h>  // Widget library

void CockpitDisplayTask(void* parameter);

void setup() {

    Serial.begin(BAUD_RATE);
    xTaskCreate(CockpitDisplayTask, "cockpitDisplay", 4096, NULL, 1, NULL);  
}

void loop() {
    vTaskDelete(NULL);
}


