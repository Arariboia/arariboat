#include <Arduino.h>
#include "Utilities.hpp"

static constexpr int pinRPMLeft = PIN_RPM_LEFT;
static constexpr int pinRPMRight = PIN_RPM_RIGHT;

static volatile unsigned long lastMeasureLeft = 0;
static volatile unsigned long periodSumLeft = 0;
static volatile unsigned int counterLeft = 0;

static volatile unsigned long lastMeasureRight = 0;
static volatile unsigned long periodSumRight = 0;
static volatile unsigned int counterRight = 0;


static bool shouldBlock = false;

static float PeriodToFrequency(float value) {
	return value == 0 ?  0 : 1000000.0 / value;	
}

static void ResetValues() {
	shouldBlock = true;
	periodSumLeft = 0;
	periodSumRight = 0;
	counterLeft = 0;
	counterRight = 0;
	shouldBlock = false;
}

static void handleFrequencyInterruptLeft() 
{
    if (shouldBlock) return;
	
    unsigned long current = micros();
    periodSumLeft += current - lastMeasureLeft;
    lastMeasureLeft = current;
    counterLeft++;
}

static void handleFrequencyInterruptRight() {

	if (shouldBlock) return;
    
    unsigned long current = micros();
    periodSumRight += current - lastMeasureRight;
    lastMeasureRight = current;
    counterRight++;	
}

static float FrequencyToRPM(float frequency) {
    constexpr float conversion_factor_slope = 15.0; // Conversion factor from frequency to RPM
    constexpr float conversion_factor_offset = 0.0; // Offset for the conversion factor

    return LinearCorrection(frequency, conversion_factor_slope, conversion_factor_offset);
}

static void UpdateSystemData(float rpm_left_motor, float rpm_right_motor) {

    SystemData::getInstance().all_info.rpm_left = rpm_left_motor;
    SystemData::getInstance().all_info.rpm_right = rpm_right_motor;
} 

static float CalculatePeriod(unsigned long periodSum, unsigned int counter) {
    return counter == 0 ? 0 : (float)periodSum / counter;
}

static void PrintReadings(float rpm_left, float rpm_right) {
    DEBUG_PRINTF("\n[RPM]Left: %.0f\tRight: %.0f\n", rpm_left, rpm_right);
}

static void HandleReadings() {
    
    float periodLeft = CalculatePeriod(periodSumLeft, counterLeft);
    float periodRight = CalculatePeriod(periodSumRight, counterRight);

    float frequencyLeft = PeriodToFrequency(periodLeft);
    float frequencyRight = PeriodToFrequency(periodRight);

    float rpmLeftMotor = FrequencyToRPM(frequencyLeft);
    float rpmRightMotor = FrequencyToRPM(frequencyRight);

    UpdateSystemData(rpmLeftMotor, rpmRightMotor);

    PrintReadings(rpmLeftMotor, rpmRightMotor);

    ResetValues();
}

void FrequencyCounterTask(void* parameter) {
    
    pinMode(pinRPMLeft, INPUT);
    pinMode(pinRPMRight, INPUT);

    attachInterrupt(digitalPinToInterrupt(pinRPMLeft), handleFrequencyInterruptLeft, RISING);
    attachInterrupt(digitalPinToInterrupt(pinRPMRight), handleFrequencyInterruptRight, RISING);

    constexpr int printInterval = 1250;
    unsigned long lastTime = 0;

    while (true) {
        
        if (millis() - lastTime >= printInterval) {
            lastTime = millis();
            HandleReadings();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
