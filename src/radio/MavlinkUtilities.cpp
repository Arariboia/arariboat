#include "MavlinkUtilities.hpp"
#include <arariboat/mavlink.h>
#include "Utilities.hpp"

#define NO_TIMESTAMP 0

// Forward declarations for functions in this file
void PrintMavlinkMessageInfo(mavlink_message_t message);
bool GenerateMavlinkMessage(mavlink_message_t& message, int message_id);
String GetMavlinkMessageName(mavlink_message_t message);
void WriteMessageToSerial(mavlink_message_t message);

/**
 * @brief Prints information about a received MAVLink message to the debug console.
 *
 * @param message The received MAVLink message.
 */
void PrintMavlinkMessageInfo(mavlink_message_t message) {
    DEBUG_PRINTF("[RX] Received message with ID #%d, name: %s\n", message.msgid, GetMavlinkMessageName(message).c_str());
}

/**
 * @brief Generates a MAVLink message with dummy data based on the given message ID.
 *
 * @param message Reference to the mavlink_message_t object to be populated.
 * @param message_id The ID of the message to generate.
 * @return True if the message was generated successfully, false otherwise.
 */
bool GenerateMavlinkMessage(mavlink_message_t& message, int message_id) {
    // Default system and component IDs
    uint8_t system_id = 1;
    uint8_t component_id = 200;

    switch (message_id) {

        case MAVLINK_MSG_ID_INSTRUMENTATION: {
            uint16_t temps[4] = {50, 60, 70, 90};
            mavlink_msg_instrumentation_pack(system_id, component_id, &message, 1500, 1000, 1001, 500, temps, 100, 4800, 1200, 800, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_TEMPERATURES: {
            mavlink_msg_temperatures_pack(system_id, component_id, &message, 2500, 2600, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_GPS: {
            mavlink_msg_gps_pack(system_id, component_id, &message, -220000000, -470000000, 500, 90, 90, 9, 120, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_MPPT: {
            mavlink_msg_mppt_pack(system_id, component_id, &message, 6000, 500, 4800, 250, NO_TIMESTAMP, 0);
            return true;
        }
        
        case MAVLINK_MSG_ID_MPPT_STATE: {
            mavlink_msg_mppt_state_pack(system_id, component_id, &message, MPPT_STATUS_NORMAL, MPPT_CHARGING_STATUS_BOOST, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_BMS: {
            uint16_t voltages[16] = {3300, 3301, 3302, 3303, 3304, 3305, 3306, 3307, 3308, 3309, 3310, 3311, 3312, 3313, 3314, 3315};
            int16_t temperatures[2] = {2550, 2650};
            mavlink_msg_bms_pack(system_id, component_id, &message, voltages, temperatures, 150, 85, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_BMS_STATUS: {
            int16_t temperatures[2] = {2550, 2650};
            uint8_t status_flags = CHARGE_DISCHARGE_STATE_CHARGING | CHARGE_DISCHARGE_CHARGE_MOS_ON;
            mavlink_msg_bms_status_pack(system_id, component_id, &message, temperatures, status_flags, 0, 0, 0, 0, 0, 0, 0, 0, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I: {
            mavlink_msg_ezkontrol_mcu_meter_data_i_pack(system_id, component_id, &message, 480, 150, 3000, 50, 0, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II: {
            mavlink_msg_ezkontrol_mcu_meter_data_ii_pack(system_id, component_id, &message, 45, 55, EZKONTROL_GEAR_D1, 0, 0, 0, 1, 0, NO_TIMESTAMP, 0);
            return true;
        }
        
        case MAVLINK_MSG_ID_PUMPS: {
            mavlink_msg_pumps_pack(system_id, component_id, &message, 1, NO_TIMESTAMP, 0); // Bit 0 for Pump Left
            return true;
        }

        case MAVLINK_MSG_ID_DEBUG: {
            mavlink_msg_debug_pack(system_id, component_id, &message, 12345, 0, 3.14159);
            return true;
        }
        
        default: {
            return false;
        }
    }
}

/**
 * @brief Retrieves the string name of a MAVLink message.
 *
 * @param message The MAVLink message.
 * @return A String containing the name of the message, or "UNKNOWN" if the ID is not recognized.
 */
String GetMavlinkMessageName(mavlink_message_t message) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_INSTRUMENTATION: return "INSTRUMENTATION";
        case MAVLINK_MSG_ID_TEMPERATURES: return "TEMPERATURES";
        case MAVLINK_MSG_ID_GPS: return "GPS";
        case MAVLINK_MSG_ID_MPPT: return "MPPT";
        case MAVLINK_MSG_ID_MPPT_STATE: return "MPPT_STATE";
        case MAVLINK_MSG_ID_BMS: return "BMS";
        case MAVLINK_MSG_ID_BMS_STATUS: return "BMS_STATUS";
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I: return "EZKONTROL_MCU_METER_DATA_I";
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II: return "EZKONTROL_MCU_METER_DATA_II";
        case MAVLINK_MSG_ID_PUMPS: return "PUMPS";
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ: return "PARAM_REQUEST_READ";
        case MAVLINK_MSG_ID_PARAM_VALUE: return "PARAM_VALUE";
        case MAVLINK_MSG_ID_PARAM_SET: return "PARAM_SET";
        case MAVLINK_MSG_ID_RADIO_STATUS: return "RADIO_STATUS";
        case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT: return "NAMED_VALUE_FLOAT";
        case MAVLINK_MSG_ID_NAMED_VALUE_INT: return "NAMED_VALUE_INT";
        case MAVLINK_MSG_ID_STATUSTEXT: return "STATUSTEXT";
        case MAVLINK_MSG_ID_DEBUG: return "DEBUG";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Serializes a MAVLink message and writes it to the Serial port.
 *
 * @param message The MAVLink message to be sent.
 */
void WriteMessageToSerial(mavlink_message_t message) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    Serial.write(buffer, len);
}

void LineProtocolAddTag(char* buffer, const char* key, const char* value) {
    sprintf(buffer + strlen(buffer), "%s=%s,", key, value);
}

void LineProtocolAddField(char* buffer, const char* key, float value) {
    sprintf(buffer + strlen(buffer), "%s=%.2f,", key, value);
}

void LineProtocolAddField(char* buffer, const char* key, int value) {
    sprintf(buffer + strlen(buffer), "%s=%d,", key, value);
}

void LineProtocolAddTimestamp(char* buffer, uint32_t timestamp_s, uint16_t timestamp_ms) {
    unsigned long timestamp = timestamp_s * 1000 + timestamp_ms;
    sprintf(buffer + strlen(buffer), " %lu", timestamp);
}

void MavlinkEzkontrol_I_toLineProtocol(char *buffer, mavlink_ezkontrol_mcu_meter_data_i_t data) {
    LineProtocolAddTag(buffer, "message", "motorEletricalData");
    LineProtocolAddTag(buffer, "instance", data.instance == 0 ? "left" : "right");
    sprintf(buffer + strlen(buffer), " ");
    LineProtocolAddField(buffer, "busVoltage", data.bus_voltage);
    LineProtocolAddField(buffer, "busCurrent", data.bus_current);
    LineProtocolAddField(buffer, "rpm", data.rpm);
    LineProtocolAddField(buffer, "acceleratorOpening", data.accelerator_opening);
    LineProtocolAddTimestamp(buffer, data.timestamp_seconds, data.timestamp_milliseconds);
}

void MavlinkEzkontrol_II_toLineProtocol(char *buffer, mavlink_ezkontrol_mcu_meter_data_ii_t data) {
    LineProtocolAddTag(buffer, "message", "motorStateData");
    LineProtocolAddTag(buffer, "instance", data.instance == 0 ? "left" : "right");
    sprintf(buffer + strlen(buffer), " ");
    LineProtocolAddField(buffer, "controllerTemperature", data.controller_temperature);
    LineProtocolAddField(buffer, "motorTemperature", data.motor_temperature);
    LineProtocolAddTimestamp(buffer, data.timestamp_seconds, data.timestamp_milliseconds);
}

void MavlinkBMSToLineProtocol(char *buffer, mavlink_bms_t data){
    LineProtocolAddTag(buffer, "message", "bms");
    sprintf(buffer + strlen(buffer), " ");
    LineProtocolAddField(buffer, "batteryCurrent", data.current_battery);
    LineProtocolAddField(buffer, "stateOfCharge", data.state_of_charge);
    LineProtocolAddField(buffer, "temperature1", data.temperatures[0]);
    LineProtocolAddField(buffer, "temperature2", data.temperatures[1]);
    LineProtocolAddField(buffer, "voltageCell1", data.voltages[0]);
    LineProtocolAddField(buffer, "voltageCell2", data.voltages[1]);
    LineProtocolAddField(buffer, "voltageCell3", data.voltages[2]);
    LineProtocolAddField(buffer, "voltageCell4", data.voltages[3]);
    LineProtocolAddField(buffer, "voltageCell5", data.voltages[4]);
    LineProtocolAddField(buffer, "voltageCell6", data.voltages[5]);
    LineProtocolAddField(buffer, "voltageCell7", data.voltages[6]);
    LineProtocolAddField(buffer, "voltageCell8", data.voltages[7]);
    LineProtocolAddField(buffer, "voltageCell9", data.voltages[8]);
    LineProtocolAddField(buffer, "voltageCell10", data.voltages[9]);
    LineProtocolAddField(buffer, "voltageCell11", data.voltages[10]);
    LineProtocolAddField(buffer, "voltageCell12", data.voltages[11]);
    LineProtocolAddField(buffer, "voltageCell13", data.voltages[12]);
    LineProtocolAddField(buffer, "voltageCell14", data.voltages[13]);
    LineProtocolAddField(buffer, "voltageCell15", data.voltages[14]);
    LineProtocolAddField(buffer, "voltageCell16", data.voltages[15]);
    LineProtocolAddTimestamp(buffer, data.timestamp_seconds, data.timestamp_milliseconds);
}

void MavlinkMPPTToLineProtocol(char *buffer, mavlink_mppt_t data){
    LineProtocolAddTag(buffer, "message", "mppt");
    sprintf(buffer + strlen(buffer), " ");
    LineProtocolAddField(buffer, "panelVoltage", data.pv_voltage);
    LineProtocolAddField(buffer, "panelCurrent", data.pv_current);
    LineProtocolAddField(buffer, "batteryVoltage", data.battery_voltage);
    LineProtocolAddField(buffer, "batteryCurrent", data.battery_current);
    LineProtocolAddTimestamp(buffer, data.timestamp_seconds, data.timestamp_milliseconds);
}

String MavlinkToLineProtocol(mavlink_message_t message) {
    char buffer[5112];
    memset(buffer, 0, sizeof(buffer));
    sprintf(buffer, "Yonah,");
    switch (message.msgid) {
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_I: {
            mavlink_ezkontrol_mcu_meter_data_i_t motor_data;
            mavlink_msg_ezkontrol_mcu_meter_data_i_decode(&message, &motor_data);
            MavlinkEzkontrol_I_toLineProtocol(buffer, motor_data);
            break;
        }
        case MAVLINK_MSG_ID_EZKONTROL_MCU_METER_DATA_II: {
            mavlink_ezkontrol_mcu_meter_data_ii_t motor_data;
            mavlink_msg_ezkontrol_mcu_meter_data_ii_decode(&message, &motor_data);
            MavlinkEzkontrol_II_toLineProtocol(buffer, motor_data);
            break;
        }
        case MAVLINK_MSG_ID_BMS: {
            mavlink_bms_t bms_data;
            mavlink_msg_bms_decode(&message, &bms_data);
            MavlinkBMSToLineProtocol(buffer, bms_data);
            break;
        }
        case MAVLINK_MSG_ID_MPPT: {
            mavlink_mppt_t mppt_data;
            mavlink_msg_mppt_decode(&message, &mppt_data);
            MavlinkMPPTToLineProtocol(buffer, mppt_data);
            break;
        }
        default: {
            sprintf(buffer, "");
            break;
        }
    }
    return String(buffer);
}