#include <esp_event.h> // ESP-IDF event loop library for handling events between tasks

//Define a user event loop to handle events between tasks.
//This allows tasks to communicate with each other with loosely coupled code.
extern esp_event_loop_handle_t eventLoop; // Tasks can use this to post and subscribe to events
ESP_EVENT_DECLARE_BASE(COMMAND_BASE); // Subscribers can receive serial parser events here

void initialize_event_loop(esp_event_loop_handle_t* eventLoop);
