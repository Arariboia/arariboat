#include "parameter.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"


// Arguments for the "param_get" command
static struct {
    struct arg_str *name; // Parameter name
    struct arg_end *end; // End of arguments
} get_param_args;

// Arguments for the "param_set" command
static struct {   
    struct arg_str *name; // Parameter name
    struct arg_dbl *value; // New value for the parameter
    struct arg_end *end; // End of arguments
} set_param_args;

// ---------------Handler functions----------------------------

/*
* @brief Handler for the "param_list" command.
*/
static int param_list_command_handler(int argc, char **argv) {
    int count = parameter_manager.get_param_count();
    const int buffer_size = 64 + 40 * count; // Estimate buffer size based on number of parameters
    char buffer[buffer_size]; // Buffer to hold the entire output string
    int len = 0; // Current length of the string in the buffer
    len += snprintf(buffer + len, sizeof(buffer) - len, "--- Registered Parameters ---\n");

    if (count == 0) {
        printf("No parameters registered.\n");
        return 0; // No parameters to list
    }

    for (int i = 0; i < count; ++i) {
        Parameter param;
        if (parameter_manager.get_param_by_index(i, param)) {
            len += snprintf(buffer + len, sizeof(buffer) - len, "%-16s = %.2f (Default: %.2f)\n", 
                            param.key, *param.value_ptr, param.defaultValue);
        }
    }

    len += snprintf(buffer + len, sizeof(buffer) - len, "--- End of Parameters ---\n");

    if (len > sizeof(buffer) - 1) {
        len = sizeof(buffer) - 1; // Ensure we don't overflow the buffer
    }
    buffer[len] = '\0'; // Null-terminate the string
    printf("%s", buffer); // Print the entire buffer at once
}

/**
 * @brief Handler for the "param_get" command.
 */
static int param_get_command_handler(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&get_param_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, get_param_args.end, argv[0]);
        return 1; // Return error code
    }

    const char *param_name = get_param_args.name->sval[0];
    Parameter param;

    if (parameter_manager.get_param_by_name(param_name, param)) {
        printf("Parameter '%s' = %.2f (Default: %.2f)\n", 
               param.key, *param.value_ptr, param.defaultValue);
        return 0; // Success
    } else {
        printf("Parameter '%s' not found.\n", param_name);
        return 1; // Parameter not found
    }
}

/**
 * @brief Handler for the "param_set" command.
 */
static int param_set_command_handler(int argc, char **argv) {
    int nerrors = arg_parse(argc, argv, (void **)&set_param_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, set_param_args.end, argv[0]);
        return 1; // Return error code
    }

    const char *param_name = set_param_args.name->sval[0];
    float new_value = set_param_args.value->dval[0];

    if (parameter_manager.set_param_by_name(param_name, new_value)) {
        printf("Parameter '%s' set to %.2f\n", param_name, new_value);
        parameter_manager.save_all_params(); // Save changes to persistent storage
        return 0; // Success
    } else {
        printf("Parameter '%s' not found.\n", param_name);
        return 1; // Parameter not found
    }
}

/*
* @brief Handles the 'param_save' command to save all parameters to persistent storage.
*/
static int param_save_command_handler(int argc, char **argv) {
    parameter_manager.save_all_params();
    return 0; // Success
}

// --- Command Registration ---
void register_parameter_commands() {
    // --- Register 'param_list' ---
    const esp_console_cmd_t list_cmd = {
        .command = "param_list",
        .help = "List all registered parameters and their current values",
        .hint = NULL,
        .func = &param_list_command_handler,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&list_cmd));

    // --- Register 'param_get' ---
    get_param_args.name = arg_str1(NULL, NULL, "<name>", "Name of the parameter to get");
    get_param_args.end = arg_end(2);
    const esp_console_cmd_t get_cmd = {
        .command = "param_get",
        .help = "Get the value of a parameter by its name",
        .hint = NULL,
        .func = &param_get_command_handler,
        .argtable = &get_param_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&get_cmd));

    // --- Register 'param_set' ---
    set_param_args.name = arg_str1(NULL, NULL, "<name>", "Name of the parameter to set");
    set_param_args.value = arg_dbl1(NULL, NULL, "<value>", "The new floating-point value");
    set_param_args.end = arg_end(2);
    const esp_console_cmd_t set_cmd = {
        .command = "param_set",
        .help = "Set the value of a parameter",
        .hint = NULL,
        .func = &param_set_command_handler,
        .argtable = &set_param_args
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&set_cmd));

    // --- Register 'param_save' ---
    const esp_console_cmd_t save_cmd = {
        .command = "param_save",
        .help = "Save all current parameter values to persistent storage",
        .hint = NULL,
        .func = &param_save_command_handler,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&save_cmd));
    
    printf("[PARAM] Parameter commands registered\n");
}