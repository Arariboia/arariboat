#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Adafruit_ADS1X15.h" // 16-bit high-linearity with programmable gain amplifier Analog-Digital Converter for measuring current and voltage.
#include <SPI.h> // Required for the ADS1115 ADC.
#include <Wire.h> // Required for the ADS1115 ADC and communication with the LoRa board.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.

/* 
The ADS1115 is a Delta-sigma (ΔΣ) ADC, based on the principle of
oversampling. The input signal of a ΔΣ ADC is sampled at a high
frequency (modulator frequency) and subsequently filtered and
decimated in the digital domain to yield a conversion result at the
respective output data rate.

The ratio between modulator frequency and output data rate is called
oversampling ratio (OSR). By increasing the OSR, and thus reducing the
output data rate, the noise performance of the ADC can be optimized.
In other words, the input-referred noise drops when reducing the output
data rate because more samples of the internal modulator are averaged
to yield one conversion result. Increasing the gain, therefore reducing
the input voltage range, also reduces the input-referred noise, which
is particularly useful when measuring low-level signals.

The use of the ADS1115 as an external ADC was chosen to obtain higher
resolution and linearity, as well as programmable gain to avoid the
need for instrumentation amplifiers. The ADS1115 has 4 addresses, which
are determined by the state of the ADDR pin. Our board has a solder
bridge / jumper that allows selection between addresses. The ADS1115 is
connected to the ESP32 via I2C. The ESP32 is the master and the ADS1115
is the slave. Make sure that the ADS1115 is connected to the ESP32 via
I2C and that it is set to a correct address.

A common ground is also required between the ESP32 and the ADS1115,
which is given when the ESP32 is powered by the same battery as the
ADS1115, but not when the ESP32 is powered by the USB port during tests
on the laboratory workbench. In this case, the ground of the ESP32 and
the ADS1115 must be explicitly connected together for the I2C
communication to work. If the ADS1115 is not detected, check continuity
of the wires with a multimeter.
*/

typedef Adafruit_ADS1115 ADS1115; // Alias for the ADS1115 class.

static void commandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (STRINGS_ARE_EQUAL(command, "adc")) {

        ADS1115* adc = (ADS1115*)handler_args;

        Serial.printf("\n[Instrumentation]Reading ADC values\n");
        adc->setGain(GAIN_FOUR);
        float voltage_battery = LinearCorrection(adc->readADC_SingleEnded(0), 0.002472f, 0.801442);

        adc->setGain(GAIN_EIGHT);
        float current_port = LinearCorrection(adc->readADC_SingleEnded(1), 0.005653f, -56.366843f);

        adc->setGain(GAIN_EIGHT);
        float current_starboard = LinearCorrection(adc->readADC_SingleEnded(2), 0.005627f, -56.204637f);

        adc->setGain(GAIN_EIGHT);
        float current_mppt = LinearCorrection(adc->readADC_SingleEnded(3), 0.001602f, 0.015848f);

        Serial.printf("\n[Instrumentation]Battery voltage: %.2fV\n"
                      "[Instrumentation]Port current: %.2fA\n"
                      "[Instrumentation]Starboard current: %.2fA\n"
                      "[Instrumentation]MPPT current: %.2fA\n",
                      voltage_battery, current_port, current_starboard, current_mppt);
        
    }
}

bool TryInitializeIrradiance(ADS1115& adc, int address) {
    static bool initialized = false;
    static bool firstPrint = false;
    if (!initialized) {
        initialized = adc.begin(address);
    }

    if (!initialized) return false;
    
    if (!firstPrint) {
        DEBUG_PRINTF("\n[ADS]IRRADIANCE successfully initialized at address 0x%x\n", address);
        firstPrint = true;
    }

    return true;
}

float CalculateIrradiance(ADS1115& adc) {
    int digital_output = adc.readADC_SingleEnded(0);
    float voltage = adc.computeVolts(digital_output);
    
    constexpr float voltage_to_irradiance = 1000.0f / 0.224f;
    float irradiance = voltage * voltage_to_irradiance;
   
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 5000) {
        lastTime = millis();
        DEBUG_PRINTF("\n[INSTRUMENTATION]ADC: %d\tVoltage: %fV\tIrradiance: %.0fW/m2\n", digital_output, voltage, irradiance);
    }

    return irradiance;
}

void InstrumentationTask(void* parameter) {
    
    Wire.begin(); // I2C master mode to communicate with the ADS1115 ADC

    ADS1115 adc;
    constexpr uint8_t adc_addresses[] = {0x48}; // Address is determined by a solder bridge or jumper on the instrumentation board.
    adc.setGain(GAIN_FOUR); // Should be set such that the maximum voltage of the input signal is close to the selected full-scale input voltage.
    adc.setDataRate(RATE_ADS1115_16SPS); // Setting a low data rate to increase the oversampling ratio of the ADC and thus reduce the noise.
    
    bool is_adc_initialized = false;
    
    while (!is_adc_initialized) {
        for (auto& address : adc_addresses) {
            if (adc.begin(address)) {
                DEBUG_PRINTF("\n[ADS]ADS1115 successfully initialized at address 0x%x\n", address);
                is_adc_initialized = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(400));
        }
    }

    ADS1115 irradianceADC;
    irradianceADC.setGain(GAIN_SIXTEEN);
    irradianceADC.setDataRate(RATE_ADS1115_16SPS);


    //Register serial callback commands
    esp_event_handler_register_with(eventLoop, COMMAND_BASE, ESP_EVENT_ANY_ID, commandCallback, &adc);
    
    while (true) {
        // In the ADS1115 single ended measurements have 15 bits of resolution. Only differential measurements have 16 bits of resolution.
        // As we are using the 4 analog inputs for each of the 4 sensors, single ended measurements are being used in order to access all 4 sensors.
        // When using single ended mode, the maximum output code is 0x7FFF(32767), which corresponds to the full-scale input voltage.
        
        if (TryInitializeIrradiance(irradianceADC, 0x49)) {
            CalculateIrradiance(irradianceADC);
        }

        adc.setGain(GAIN_FOUR);
        float voltage_battery = LinearCorrection(adc.readADC_SingleEnded(0), 0.002472f, 0.801442);

        adc.setGain(GAIN_EIGHT);
        float current_port = LinearCorrection(adc.readADC_SingleEnded(1), 0.005653f, -56.366843f);

        adc.setGain(GAIN_EIGHT);
        float current_starboard = LinearCorrection(adc.readADC_SingleEnded(2), 0.005627f, -56.204637f);

        current_starboard = current_starboard - current_port;

        adc.setGain(GAIN_EIGHT);
        float current_mppt = LinearCorrection(adc.readADC_SingleEnded(3), 0.001602f, 0.015848f);

        DEBUG_PRINTF("\n[Instrumentation]Battery voltage: %.2fV\n"
                      "[Instrumentation]Port current: %.2fA\n"
                      "[Instrumentation]Starboard current: %.2fA\n"
                      "[Instrumentation]MPPT current: %.2fA\n",
                      voltage_battery, current_port, current_starboard, current_mppt);
        
        SystemData::getInstance().all_info.battery_voltage = voltage_battery;
        SystemData::getInstance().all_info.motor_current_left = current_port;
        SystemData::getInstance().all_info.motor_current_right = current_starboard;
        SystemData::getInstance().all_info.mppt_current = current_mppt;
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


