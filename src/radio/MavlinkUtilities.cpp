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
            mavlink_msg_instrumentation_pack(system_id, component_id, &message, 1500, 1000, 1001, 500, 100, 4800, 1200, 800, NO_TIMESTAMP, 0);
            return true;
        }

        case MAVLINK_MSG_ID_TEMPERATURES: {
            mavlink_msg_temperatures_pack(system_id, component_id, &message, 2500, 2600, 3500, 3600, NO_TIMESTAMP, 0);
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