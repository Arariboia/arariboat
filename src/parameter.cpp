#include "parameter.h"

ParameterManager parameter_manager;

void ParameterManager::begin() {
    #define READ_WRITE false
    preferences.begin("boat-params", READ_WRITE);

    char buffer[128];
    int index = 0;
    memset(buffer, 0, sizeof(buffer));

    index += snprintf(buffer + index, sizeof(buffer) - index, "[PARAM] %d parameters registered\n", param_list.size());
    if (param_list.empty() || !preferences.isKey(param_list[0].key)) {
        snprintf(buffer + index, sizeof(buffer) - index, "[PARAM] First boot or no parameters found, resetting to defaults\n");
        printf("%s", buffer);
        reset_all_to_defaults();
    } else {
        snprintf(buffer + index, sizeof(buffer) - index, "[PARAM] Loading parameters from persistent storage\n");
        printf("%s", buffer);
        load_all_params();
    }
}

void ParameterManager::register_param(const char* key, float* value_ptr, float default_value) {
    assert(value_ptr != nullptr); // Ensure a valid address is provided
    assert(strlen(key) < sizeof(Parameter::key)); // Ensure key length does not exceed buffer size
    Parameter new_param;
    strncpy(new_param.key, key, sizeof(new_param.key) - 1);
    new_param.key[sizeof(new_param.key) - 1] = '\0'; // Ensure null termination
    new_param.value_ptr = value_ptr;
    new_param.defaultValue = default_value;
    param_list.push_back(new_param);
}

int ParameterManager::get_param_count() const {
    return param_list.size();
}

bool ParameterManager::get_param_by_index(int index, Parameter& param_out) const {
    if (index < 0 || index >= param_list.size()) {
        return false; // Index out of bounds
    }
    param_out = param_list[index];
    return true; // Successfully retrieved parameter
}

bool ParameterManager::get_param_by_name(const char* key, Parameter& param_out) const {
    for (const auto &param : param_list) {
        if (strcmp(param.key, key) == 0) {
            param_out = param; // Fill the output parameter struct
            return true; // Parameter found
        }
    }
    return false; // Parameter not found
}

bool ParameterManager::set_param_by_name(const char* key, float value) {
    for (auto &param : param_list) {
        if (strcmp(param.key, key) == 0) {
            *param.value_ptr = value; // Set the value
            return true;
        }
    }
    return false; // Parameter not found
}

void ParameterManager::load_all_params() {
    for (auto &param :param_list) {
        *param.value_ptr = preferences.getFloat(param.key, param.defaultValue);
    }
}

void ParameterManager::save_all_params() {
    printf("[PARAM]Saving parameters to persistent storage...\n");
    for (const auto &param : param_list) {
        preferences.putFloat(param.key, *param.value_ptr);
    }
    printf("[PARAM] All parameters saved to persistent storage\n");
}

void ParameterManager::reset_all_to_defaults() {
    printf("[PARAM] Resetting all parameters to defaults...\n");
    for (auto &param : param_list) {
        *param.value_ptr = param.defaultValue; // Reset to default value
        preferences.putFloat(param.key, param.defaultValue); // Save default value to persistent storage
    }
    printf("[PARAM] All parameters reset to defaults\n");
}
