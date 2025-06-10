#pragma once
#include "Utilities.hpp"

enum BlinkRate : uint32_t {
    Slow = 2000,
    Medium = 1000,
    Fast = 300,
    Pulse = 100 // Pulse is a special value that will make the LED blink fast and then return to the previous blink rate.
};

struct BlinkCommand {
    long offTime;
    long onTime;
    int repeats;
};

void BlinkNotify(const char* command);
