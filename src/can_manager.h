#pragma once
#include "driver/twai.h"

class ICANManager {
public:
    virtual bool handle_can_frame(const twai_message_t& message) = 0;
    virtual ~ICANManager() = default;
};
