#pragma once

#include <stdint.h>
#include <cstring>
#include <map>
#include <string>

// Parameter types
enum ap_param_type {
    AP_PARAM_NONE = 0,
    AP_PARAM_INT8,
    AP_PARAM_INT16,
    AP_PARAM_INT32,
    AP_PARAM_FLOAT,
};

// Base parameter class
class AP_Param {
public:
    static bool load_all();
    static bool save_all();
};

// Float parameter
class AP_ParamFloat : public AP_Param {
public:
    AP_ParamFloat(float default_val, const char* name) : value(default_val), param_name(name) {
        register_param(name, this);
    }

    operator float() const { return value; }

    AP_ParamFloat& operator=(float val) {
        value = val;
        return *this;
    }

    float get() const { return value; }
    void set(float val) { value = val; }
    void set_and_save(float val) {
        value = val;
        // In a full implementation, this would save to EEPROM/flash
    }

    const char* get_name() const { return param_name; }

private:
    float value;
    const char* param_name;

    static void register_param(const char* name, AP_ParamFloat* param);
    static std::map<std::string, AP_ParamFloat*>& get_param_map();

    friend class AP_Param;
};

// Int16 parameter
class AP_ParamInt16 : public AP_Param {
public:
    AP_ParamInt16(int16_t default_val, const char* name) : value(default_val), param_name(name) {
        register_param(name, this);
    }

    operator int16_t() const { return value; }

    AP_ParamInt16& operator=(int16_t val) {
        value = val;
        return *this;
    }

    int16_t get() const { return value; }
    void set(int16_t val) { value = val; }
    const char* get_name() const { return param_name; }

private:
    int16_t value;
    const char* param_name;

    static void register_param(const char* name, AP_ParamInt16* param);
    static std::map<std::string, AP_ParamInt16*>& get_param_map();

    friend class AP_Param;
};

// Int8 parameter
class AP_ParamInt8 : public AP_Param {
public:
    AP_ParamInt8(int8_t default_val, const char* name) : value(default_val), param_name(name) {
        register_param(name, this);
    }

    operator int8_t() const { return value; }

    AP_ParamInt8& operator=(int8_t val) {
        value = val;
        return *this;
    }

    int8_t get() const { return value; }
    void set(int8_t val) { value = val; }
    const char* get_name() const { return param_name; }

private:
    int8_t value;
    const char* param_name;

    static void register_param(const char* name, AP_ParamInt8* param);
    static std::map<std::string, AP_ParamInt8*>& get_param_map();

    friend class AP_Param;
};
