#pragma once
#include "arariboat/mavlink.h"
#include "data.hpp" // Include the data structures used in the conversion functions
// Forward declaration for the conversion functions
bool message_t_from_mavlink_msg(const mavlink_message_t *mavlink_msg, message_t *message);
bool mavlink_msg_from_message_t(message_t message, mavlink_message_t *mavlink_msg);
void print_received_mavlink_message(const mavlink_message_t* msg);