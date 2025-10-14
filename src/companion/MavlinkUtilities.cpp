#include "Utilities.hpp"
#include "MavlinkUtilities.hpp"
#define NO_TIMESTAMP 0

// void PrintMavlinkMessageInfo(mavlink_message_t message) {
//     switch (message.msgid) {
//         case MAVLINK_MSG_ID_HEARTBEAT: {
//             DEBUG_PRINTF("[SERIAL]Received heartbeat\n", NULL);
//             break;
//         }
//         case MAVLINK_MSG_ID_INSTRUMENTATION: {
//             DEBUG_PRINTF("[SERIAL]Received instrumentation\n", NULL);
//             break;
//         }
//         case MAVLINK_MSG_ID_TEMPERATURES: {
//             DEBUG_PRINTF("[SERIAL]Received temperatures\n", NULL);
//             break;
//         }
//         case MAVLINK_MSG_ID_GPS_INFO: {
//             DEBUG_PRINTF("[SERIAL]Received GPS info\n", NULL);
//             break;
//         }
//         case MAVLINK_MSG_ID_RPM_INFO: {
//             DEBUG_PRINTF("[SERIAL]Received RPM info\n", NULL);
//             break;
//         }
//         case MAVLINK_MSG_ID_ALL_INFO: {
//             DEBUG_PRINTF("[SERIAL]Received all info\n", NULL);
//             break;
//         }

//         default: {
//             DEBUG_PRINTF("[SERIAL]Received message with ID #%d\n", message.msgid);
//             break;
//         }     
//     }
// }

// bool GenerateMavlinkMessage(mavlink_message_t& message, int message_id) {
//     switch (message_id) {
//         case MAVLINK_MSG_ID_HEARTBEAT: {
//             mavlink_msg_heartbeat_pack(1, 200, &message, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_DECODE_POSITION_MANUAL, 0, MAV_STATE_ACTIVE);
//             return true;
//         }

//         case MAVLINK_MSG_ID_TEMPERATURES: {
//             mavlink_msg_temperatures_pack(1, 200, &message, 25.0, 30.0, 35.0, NO_TIMESTAMP);
//             return true;
//         }

//         case MAVLINK_MSG_ID_INSTRUMENTATION: {
//             mavlink_msg_instrumentation_pack(1, 200, &message, 25.2, 10.0, 20.0, 5.0, NO_TIMESTAMP);
//             return true;
//         }

//         case MAVLINK_MSG_ID_GPS_INFO: {
//             mavlink_msg_gps_info_pack(1, 200, &message, -22.0, -47.0, 5.0, 90.0, 9, NO_TIMESTAMP);
//             return true;
//         }

//         case MAVLINK_MSG_ID_RPM_INFO: {
//             mavlink_msg_rpm_info_pack(1, 200, &message, 45.0, 73.0, NO_TIMESTAMP);
//             return true;
//         }

//         case MAVLINK_MSG_ID_ALL_INFO: {
//             mavlink_msg_all_info_pack(1, 200, &message, 25.2, 10.0, 20.0, 5.0, -22.0, -47.0, 5.0, 90.0, 9, 45.0, 73.0, NO_TIMESTAMP);
//             return true;
//         }

//         default: {
//             return false;
//         }
//     }
// }

// String GetMavlinkMessageName(mavlink_message_t message) {
//     switch (message.msgid) {
//         case MAVLINK_MSG_ID_HEARTBEAT: {
//             return "HEARTBEAT";
//         }
//         case MAVLINK_MSG_ID_INSTRUMENTATION: {
//             return "INSTRUMENTATION";
//         }
//         case MAVLINK_MSG_ID_TEMPERATURES: {
//             return "TEMPERATURES";
//         }
//         case MAVLINK_MSG_ID_GPS_INFO: {
//             return "GPS_INFO";
//         }
//         case MAVLINK_MSG_ID_RPM_INFO: {
//             return "RPM_INFO";
//         }
//         case MAVLINK_MSG_ID_ALL_INFO: {
//             return "ALL_INFO";
//         }
//         default: {
//             return "UNKNOWN";
//         }
//     }
// }

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
    LineProtocolAddField(buffer, "controllerTemperature", data.controller_temperature);
    LineProtocolAddField(buffer, "motorTemperature", data.motor_temperature);
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
        default: {
            sprintf(buffer, "");
            break;
        }
    }
    return String(buffer);
}