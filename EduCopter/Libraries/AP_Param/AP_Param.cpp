#include "AP_Param.h"
#include <cstdio>

// Static parameter maps
std::map<std::string, AP_ParamFloat*>& AP_ParamFloat::get_param_map() {
    static std::map<std::string, AP_ParamFloat*> param_map;
    return param_map;
}

std::map<std::string, AP_ParamInt16*>& AP_ParamInt16::get_param_map() {
    static std::map<std::string, AP_ParamInt16*> param_map;
    return param_map;
}

std::map<std::string, AP_ParamInt8*>& AP_ParamInt8::get_param_map() {
    static std::map<std::string, AP_ParamInt8*> param_map;
    return param_map;
}

void AP_ParamFloat::register_param(const char* name, AP_ParamFloat* param) {
    get_param_map()[name] = param;
}

void AP_ParamInt16::register_param(const char* name, AP_ParamInt16* param) {
    get_param_map()[name] = param;
}

void AP_ParamInt8::register_param(const char* name, AP_ParamInt8* param) {
    get_param_map()[name] = param;
}

bool AP_Param::load_all() {
    // In a full implementation, this would load from EEPROM/flash
    // For now, we just use defaults
    printf("AP_Param: Using default parameters\n");
    return true;
}

bool AP_Param::save_all() {
    // In a full implementation, this would save to EEPROM/flash
    printf("AP_Param: Parameters saved (simulated)\n");
    return true;
}
