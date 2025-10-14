#include <Arduino.h>
#include "Utilities.hpp"
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.
#include <LoRa.h> // Lora physical layer library
#include "BoardDefinitions.h" // SX1276, SDCard and OLED display pin definitions
#include "radio.hpp" // Radio operation header file
#include "LoraConfigManager.hpp" // Non-volatile storage for system parameters
#include "event_loop.hpp" // Event loop for handling events
#include "queues.hpp"
#include "data.hpp"
#include "radio_manager.h"
#include "radio_defs.h"  // Radio definitions and constants
#include "mavlink_data_conversion.h" // Mavlink data conversion functions
#include "throttling_config.h"
#include "storage.hpp"

// A simple struct to pass received packet data from the ISR to the main task.
typedef struct {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    int len;
    int rssi;
} lora_packet_t;

// This queue is for passing raw LoRa packet data from the ISR to the processing task.
static QueueHandle_t lora_rx_queue = NULL;

static void SerialCommandCallback(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    
    const char* command = (const char*)event_data;

    if (strncmp(command, "lora", 4) == 0) {
        LoraConfiguration config;
        if (!GetLoraConfiguration(config)) {
            DEBUG_PRINTF("[RADIO]Failed to get Lora configuration\n", NULL);
            return;
        }

        DEBUG_PRINTF("[RADIO]Lora configuration: Freq: %ld, BW: %ld, SF: %d, CR: %d, Power: %d, SyncWord: %d, CRC: %d\n",
            config.frequency, config.bandwidth, config.spreadingFactor, config.codingRate, config.power, config.syncWord, config.crcEnabled);
    }
}

void InitializeRadio() {
    
    Lora.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    LoraConfiguration config;
    if (!GetLoraConfiguration(config)) {
        Serial.println("Failed to get Lora configuration.");
        vTaskDelete(NULL);
    }

    while (!Lora.begin(config.frequency)) { 
        Serial.println("Starting Lora failed!");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Starting Lora succeeded!");

    Lora.setSpreadingFactor(config.spreadingFactor);
    Lora.setSignalBandwidth(config.bandwidth);
    Lora.setCodingRate4(config.codingRate);
    Lora.setTxPower(config.power);
    Lora.setSyncWord(config.syncWord);
    config.crcEnabled ? Lora.enableCrc() : Lora.disableCrc();

    Serial.printf("LoRa configuration: Freq: %ld, BW: %ld, SF: %d, CR: %d, Power: %d, SyncWord: %d, CRC: %d\n",
        config.frequency, config.bandwidth, config.spreadingFactor, config.codingRate, config.power, config.syncWord, config.crcEnabled);
}

void InitializeRadioQueue() {
    constexpr int radioQueueLength = 5;
    auxiliary_radio_queue = xQueueCreate(radioQueueLength, sizeof(mavlink_message_t));

    if (auxiliary_radio_queue == NULL) {
        Serial.println("Failed to create radio queue!");
        vTaskDelete(NULL);
    }
}

/**
 * @brief ISR callback for LoRa packet reception.
 *
 * This function is an Interrupt Service Routine. It must be very fast.
 * Its only job is to read the incoming data from LoRa and post it to a queue
 * for the radio_receiver to process.
 * No heavy processing or serial printing should ever occur here.
 *
 * @param packetSize The size of the incoming LoRa packet.
 */
void IRAM_ATTR onLoraReceive(int packetSize) {
    if (packetSize == 0 || packetSize > MAVLINK_MAX_PACKET_LEN) return;

    lora_packet_t packet;
    packet.len = packetSize;
    packet.rssi = Lora.packetRssi();
    
    // Read the packet directly into the struct's buffer.
    Lora.readBytes(packet.buffer, packet.len);

    // Use the non-blocking "FromISR" version to send to the queue.
    // A higher-priority task can be woken up if it was waiting for an item.
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(lora_rx_queue, &packet, &xHigherPriorityTaskWoken);

    // If xHigherPriorityTaskWoken is now set to pdTRUE, a context switch should be requested.
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


/**
 * @brief FreeRTOS task to manage the radio receiver.
 * * This task initializes the LoRa radio for receiving, registers the onLoraReceive callback,
 * and then enters a loop. The actual work of receiving and parsing packets is handled 
 * by the onLoraReceive callback, which is triggered by an interrupt.
 * * @param parameter Task parameters (not used).
 */
void radio_receiver(void) {
    // Create the queue to receive raw packets from the LoRa ISR.
    lora_rx_queue = xQueueCreate(10, sizeof(lora_packet_t));
    if (lora_rx_queue == NULL) {
        DEBUG_PRINTF("[RADIO] Failed to create LoRa RX queue.\n", NULL);
        vTaskDelete(NULL); // Cannot proceed.
    }

    // Put the LoRa module into continuous receive mode and register the ISR.
    Lora.onReceive(onLoraReceive);
    Lora.receive();

    lora_packet_t rx_packet;
    mavlink_message_t mavlink_msg;
    static mavlink_status_t status; // Retain status between packets.

    while (true) {
        // Wait indefinitely for a packet to arrive from the ISR.
        if (xQueueReceive(lora_rx_queue, &rx_packet, portMAX_DELAY) == pdTRUE) {
            
            DEBUG_PRINTF("[RADIO] Processing packet. Size: %d, RSSI: %d dBm\n", rx_packet.len, rx_packet.rssi);

            // Print raw data in hex for debugging
            // char buffer[256] = {0};
            // for (int i = 0; i < rx_packet.len; i++) {
            //     sprintf(buffer + strlen(buffer), "%02X ", rx_packet.buffer[i]);
            // }
            // DEBUG_PRINTF("[RADIO] Raw Data: %s\n", buffer);

            // Process every byte in the received packet.
            for (int i = 0; i < rx_packet.len; i++) {
                // The mavlink_parse_char function attempts to build a MAVLink message byte by byte.
                if (mavlink_parse_char(MAVLINK_COMM_0, rx_packet.buffer[i], &mavlink_msg, &status)) {

                    // Update the RadioManager with the latest statistics.
                    RadioManager::GetInstance().update_statistics(status, rx_packet.rssi, mavlink_msg.msgid);

                    // Consolidated debug print for MAVLink message details and channel status.
                    DEBUG_PRINTF("[RADIO] MAVLink MSG ID: %d | Msgs Rcvd: %u | Drops: %u | Overruns: %u | Errors: %u\n", 
                                 mavlink_msg.msgid,
                                 status.packet_rx_success_count, 
                                 status.packet_rx_drop_count,
                                 status.buffer_overrun,
                                 status.parse_error);

                    #ifdef PRINT_MAVLINK_MESSAGES
                    // Print the received MAVLink message in a human-readable format.
                    print_received_mavlink_message(&mavlink_msg);
                    #endif
                    Serial.println(); // New line for better readability in the console.

                    // Route the received raw mavlink message to the console for the dashboard.
                    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {0};
                    uint16_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
                    Serial.write(buffer, len);


                    message_t message;
                    // Convert the parsed MAVLink message back to our internal message_t struct.
                    if (message_t_from_mavlink_msg(&mavlink_msg, &message)) {
                        // If conversion is successful, send the message to the main broker queue.
                        if (xQueueSend(broker_queue, &message, pdMS_TO_TICKS(10)) != pdTRUE) {
                            DEBUG_PRINTF("[RADIO] Failed to queue received message for processing.\n", NULL);
                        }
                    } else {
                        DEBUG_PRINTF("[RADIO] Failed to convert MAVLink message ID %d to message_t format.\n", mavlink_msg.msgid);
                    }
                }
            }
        }
    }
}

void radio_task(void* parameter) {
    
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for other tasks to initialize
    InitializeRadio();
    InitializeRadioQueue();

    esp_event_handler_register_with(eventLoop, SERIAL_PARSER_EVENT_BASE, ESP_EVENT_ANY_ID, SerialCommandCallback, NULL);

    // Array to hold the last time a message was sent for each data source.
    static uint32_t last_sent_time[NUM_THROTTLED_MESSAGES] = {0};

    #ifdef TRANSMITTER
    while (true) {
        message_t message;
        if (xQueueReceive(auxiliary_radio_queue, &message, portMAX_DELAY) == pdTRUE) {
            Serial.printf("[RADIO] Received message from source: %s\n", DATA_SOURCE_NAMES[message.source]);
            // System messages must be converted to MAVLink messages using conversion functions
            mavlink_message_t mavlink_msg;

            switch (message.source) {
                case DATA_SOURCE_BMS: {
                    if (millis() - last_sent_time[MSG_BMS] >= send_intervals_ms[MSG_BMS]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_BMS)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert BMS message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_BMS] = millis();
                    }
                    else if (millis() - last_sent_time[MSG_BMS_STATUS] >= send_intervals_ms[MSG_BMS_STATUS]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_BMS_STATUS)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert BMS status message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_BMS_STATUS] = millis();
                    }
                    break;
                }

                case DATA_SOURCE_MOTOR_LEFT: {
                    if (millis() - last_sent_time[MSG_MOTOR_I_LEFT] >= send_intervals_ms[MSG_MOTOR_I_LEFT]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert left motor I message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_MOTOR_I_LEFT] = millis();
                    }
                    else if (millis() - last_sent_time[MSG_MOTOR_II_LEFT] >= send_intervals_ms[MSG_MOTOR_II_LEFT]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert left motor II message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_MOTOR_II_LEFT] = millis();
                    }
                    break;
                }

                case DATA_SOURCE_MOTOR_RIGHT: {
                    if (millis() - last_sent_time[MSG_MOTOR_I_RIGHT] >= send_intervals_ms[MSG_MOTOR_I_RIGHT]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert right motor I message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_MOTOR_I_RIGHT] = millis();
                    }
                    else if (millis() - last_sent_time[MSG_MOTOR_II_RIGHT] >= send_intervals_ms[MSG_MOTOR_II_RIGHT]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert right motor II message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_MOTOR_II_RIGHT] = millis();
                    }
                    break;
                }

                case DATA_SOURCE_MPPT: {
                    if (millis() - last_sent_time[MSG_MPPT] >= send_intervals_ms[MSG_MPPT]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_MPPT)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert MPPT message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_MPPT] = millis();
                    }
                    else if (millis() - last_sent_time[MSG_MPPT_STATE] >= send_intervals_ms[MSG_MPPT_STATE]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_MPPT_STATE)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert MPPT state message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_MPPT_STATE] = millis();
                    }
                    break;
                }

                case DATA_SOURCE_GPS: {
                    if (millis() - last_sent_time[MSG_GPS] >= send_intervals_ms[MSG_GPS]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_GPS)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert GPS message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_GPS] = millis();
                    }
                    break;
                }

                case DATA_SOURCE_INSTRUMENTATION: {
                    if (millis() - last_sent_time[MSG_INSTRUMENTATION] >= send_intervals_ms[MSG_INSTRUMENTATION]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_INSTRUMENTATION)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert instrumentation message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_INSTRUMENTATION] = millis();
                    }
                    break;
                }

                case DATA_SOURCE_TEMPERATURES: {
                    if (millis() - last_sent_time[MSG_TEMPERATURES] >= send_intervals_ms[MSG_TEMPERATURES]) {
                        if (!mavlink_msg_from_message_t(message, &mavlink_msg, MAVLINK_MSG_ID_TEMPERATURES)) {
                            DEBUG_PRINTF("[RADIO] Failed to convert temperatures message to MAVLink format\n", NULL);
                        } 
                        last_sent_time[MSG_TEMPERATURES] = millis();
                    }
                    break;
                }

                default: {
                    DEBUG_PRINTF("[RADIO] Unknown data source: %s\n", DATA_SOURCE_NAMES[message.source]);
                    continue; // Skip unknown sources.
                }
            }

            // Send the MAVLink message over the radio
            unsigned long start_time = millis();
            if (Lora.beginPacket()) {
                uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
                uint16_t len = mavlink_msg_to_send_buffer(buffer, &mavlink_msg);
                Lora.write(buffer, len);
                Lora.endPacket();
                
                unsigned long end_time = millis();
                DEBUG_PRINTF("[RADIO] Sent MAVLink message with ID: %d, length: %d, time taken: %lu ms\n", 
                                mavlink_msg.msgid, len, end_time - start_time);
            } else {
                DEBUG_PRINTF("[RADIO] Failed to begin LoRa packet\n", NULL);
            }

            // Write to SDCard
            SaveMavlinkMessage(mavlink_msg);
        }
    }
    #else
    // Receiver task
    
    DEBUG_PRINTF("[RADIO] Setting up LoRa receiver...\n", NULL);
    radio_receiver(); // This will block and handle all incoming packets.
    #endif

}