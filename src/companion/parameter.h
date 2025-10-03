#pragma once
#include <Preferences.h> // Persistent storage for parameters
#include <vector> // For storing multiple parameters

struct Parameter {
    char key[17]; // Key for the parameter, max length 16 + null terminator
    float* value_ptr; // Pointer to the value of the parameter
    float defaultValue; // Default value for the parameter
};

class ParameterManager {
public:
    void begin();
    void register_param(const char* key, float* value_ptr, float default_value);

    // ---Public interface for parameter management---
    int get_param_count() const;
    bool get_param_by_index(int index, Parameter& param_out) const; // Fills a Parameter struct with the parameter data
    bool get_param_by_name(const char* key, Parameter& param_out) const; // Fills a Parameter struct with the parameter data by key
    bool set_param_by_name(const char* key, float value);
    void save_all_params(); // Save all parameters to persistent storage

private:
    Preferences preferences; // For persistent storage
    std::vector<Parameter> param_list; // List of parameters
    void load_all_params();
    void reset_all_to_defaults();
};

extern ParameterManager parameter_manager; // Global instance of ParameterManager to which modules can register parameters
