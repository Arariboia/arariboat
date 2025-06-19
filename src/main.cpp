#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include "Utilities.hpp" // Custom utility macros and functions.
#include "queues.hpp" // Header file for queue initialization and definitions.
#include "ESP32Time.h" // Internal RTC timer of ESP32 to keep track of time
#include "time_manager.h" // Header file for time management tasks.

//TODO: Improve server interface for configuration and debug purposes.
//TODO: Implement auxiliary battery and pumps readings using some I2C system  (DONE)
//TODO: Assign better weights to task priorities via benchmarks

//Commands that can be received from serial port or sent from other tasks
ESP_EVENT_DEFINE_BASE(COMMAND_BASE);

// --- Global Objects (Single source of truth) ---
ESP32Time RTC;
EventGroupHandle_t system_event_group;

/**
 * @brief Prints a detailed report of system statistics to the Serial monitor.
 * * This function is designed to be thread-safe by formatting the entire
 * report into a single buffer before printing it. This prevents output
 * from being interleaved if other tasks are also writing to the Serial port.
 * * It gathers and displays information about:
 * - General device info (chip model, CPU freq, reset reason)
 * - Date, Time, and Location (hardcoded as an example)
 * - System uptime
 * - Memory Usage (Heap)
 * - Current Task's Stack Usage (High Water Mark)
 * * It is a valuable tool for debugging performance and memory issues.
 */
void print_statistics() {
  // Use a single, large buffer to format the entire string. This makes the
  // function safer to call from different tasks, as the entire block of text
  // will be sent to the Serial port in one go, preventing interleaved messages.
  char stats_buffer[1024];
  int buffer_offset = 0; // Tracks the current end of the string in the buffer

  // A. General System and Device Information
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- SYSTEM STATISTICS ---\n");

  // A1. Chip Information
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Device Model        : %s rev %d\n", ESP.getChipModel(), chip_info.revision);
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Cores               : %d\n", chip_info.cores);
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "CPU Frequency       : %d MHz\n", getCpuFrequencyMhz());

  // A2. Last Reset Reason
  esp_reset_reason_t reset_reason_code = esp_reset_reason();
  const char* reset_reason_str;
  switch (reset_reason_code) {
    case ESP_RST_UNKNOWN:    reset_reason_str = "Unknown"; break;
    case ESP_RST_POWERON:    reset_reason_str = "Power on"; break;
    case ESP_RST_EXT:        reset_reason_str = "External pin"; break;
    case ESP_RST_SW:         reset_reason_str = "Software"; break;
    case ESP_RST_PANIC:      reset_reason_str = "Panic / Exception"; break;
    case ESP_RST_INT_WDT:    reset_reason_str = "Interrupt Watchdog"; break;
    case ESP_RST_TASK_WDT:   reset_reason_str = "Task Watchdog"; break;
    case ESP_RST_WDT:        reset_reason_str = "Other Watchdog"; break;
    case ESP_RST_DEEPSLEEP:  reset_reason_str = "Exiting deep sleep"; break;
    case ESP_RST_BROWNOUT:   reset_reason_str = "Brownout"; break;
    case ESP_RST_SDIO:       reset_reason_str = "SDIO"; break;
    default:                 reset_reason_str = "N/A"; break;
  }
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Last Reset Reason   : %s\n", reset_reason_str);

  // A3. Time, Date, and Location
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Location & Time ---\n");
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Timestamp           : Thursday, June 19, 2025, 10:46 AM (-03)\n");
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Location            : Niterói, RJ, Brazil\n");


  // B. Uptime
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Uptime ---\n");
  uint32_t uptime_ms = millis();
  uint32_t seconds = uptime_ms / 1000;
  uint32_t minutes = seconds / 60;
  uint32_t hours = minutes / 60;
  uint32_t days = hours / 24;
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "System Uptime       : %d days, %02d:%02d:%02d\n", days, hours % 24, minutes % 60, seconds % 60);


  // C. Memory Statistics
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Memory (Heap) ---\n");
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Total Heap Size     : %u bytes\n", ESP.getHeapSize());
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Free Heap Size      : %u bytes\n", ESP.getFreeHeap());
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Min Free Heap (LWM) : %u bytes\n", ESP.getMinFreeHeap());


  // D. Current Task Statistics
  // =================================================================
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- Current Task ---\n");
  char* task_name = pcTaskGetName(NULL); // NULL gets the current task
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Task Name           : %s\n", task_name);
  
  // The "high water mark" for the stack. The minimum free stack space since the task started.
  // This is a critical metric for sizing a task's stack.
  uint32_t stack_hwm = uxTaskGetStackHighWaterMark(NULL); 
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "Stack HWM           : %u bytes free\n", stack_hwm);
  
  buffer_offset += snprintf(stats_buffer + buffer_offset, sizeof(stats_buffer) - buffer_offset,
                           "--- END OF STATISTICS ---\n\n");

  // Print the entire formatted buffer in one atomic operation
  Serial.print(stats_buffer);
}

void setup() {
    Serial.begin(BAUD_RATE);
    initialize_event_loop(&eventLoop); // Initialize the event loop to handle events between tasks.
    initialize_queues(); // Initialize the queues used for inter-task communication.
    system_event_group = xEventGroupCreate(); // Create an event group for system-wide events.
    CREATE_TASK(led_manager_task, STACK_SIZE(2048), PRIORITY(1));
    // CREATE_TASK(SerialTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(wifi_task, STACK_SIZE(4096), PRIORITY(2));
    CREATE_TASK(server_task, STACK_SIZE(8096), PRIORITY(3));
    CREATE_TASK(time_manager_task, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(TemperatureTask, STACK_SIZE(4096), PRIORITY(1));
    // CREATE_TASK(GPSTask, STACK_SIZE(4096), PRIORITY(1));
    CREATE_TASK(InstrumentationTask, STACK_SIZE(8192), PRIORITY(3));
    CREATE_TASK(mppt_task, STACK_SIZE(4096), PRIORITY(3));
    CREATE_TASK(broker_task, STACK_SIZE(4096), PRIORITY(2));
    CREATE_TASK(can_task, STACK_SIZE(4096), PRIORITY(4)); // Create the CAN task to handle CAN communication.
    // CREATE_TASK(propulsion_task, STACK_SIZE(4096), PRIORITY(5)); // Create the propulsion task to handle propulsion system.
}

void loop() {

    vTaskDelay(pdMS_TO_TICKS(30000));
    print_statistics(); // Print system statistics to the Serial monitor for debugging and performance analysis.
}



