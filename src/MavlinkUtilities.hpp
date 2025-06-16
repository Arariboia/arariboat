#pragma once
#include <Arduino.h>
#include "arariboat/mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.

void PrintMavlinkMessageInfo(mavlink_message_t message);
void WriteMessageToSerial(mavlink_message_t message);
bool GenerateMavlinkMessage(mavlink_message_t& message, int message_id);
String GetMavlinkMessageName(mavlink_message_t message);
String MavlinkToLineProtocol(mavlink_message_t message);