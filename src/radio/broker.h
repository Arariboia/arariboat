#pragma once
#include <Arduino.h>
#include "queues.hpp" // Include the queues header to access the system queues

/**
 * @brief A broker task that receives data messages and forwards them to other tasks.
 *
 * This task acts as a central hub, decoupling data producers (like sensor tasks)
 * from data consumers (like a logger or radio transmitter). It waits for messages
 * on a central queue and dispatches them to one or more destination queues.
 *
 * @param parameter Task parameters (unused).
 */
void broker_task(void* parameter);


