/**
 * @file test_parameters.cpp
 * @brief Test suite for GCS_MAVLink_Parameters - Parameter protocol
 *
 * Tests parameter protocol message handling including:
 * - PARAM_REQUEST_LIST message handling
 * - PARAM_REQUEST_READ message handling
 * - PARAM_SET message handling
 * - PARAM_VALUE message generation
 * - Parameter streaming
 * - Parameter type handling (float, int32, uint32, int16, uint16, int8, uint8)
 * - Parameter bounds checking
 *
 * @author EduCopter Test Suite
 * @date 2025
 */

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cmath>
#include "mavlink_stubs.h"

// Test result codes
#define TEST_PASS() return true
#define TEST_FAIL(msg) do { printf("  FAIL: %s\n", msg); return false; } while(0)
#define TEST_ASSERT(cond, msg) if (!(cond)) TEST_FAIL(msg)

//=============================================================================
// Parameter Storage
//=============================================================================

enum MAV_PARAM_TYPE_EXTENDED {
    MAV_PARAM_TYPE_UINT8 = 1,
    MAV_PARAM_TYPE_INT8 = 2,
    MAV_PARAM_TYPE_UINT16 = 3,
    MAV_PARAM_TYPE_INT16 = 4,
    MAV_PARAM_TYPE_UINT32 = 5,
    MAV_PARAM_TYPE_INT32 = 6,
    MAV_PARAM_TYPE_UINT64 = 7,
    MAV_PARAM_TYPE_INT64 = 8,
    MAV_PARAM_TYPE_REAL32 = 9,
    MAV_PARAM_TYPE_REAL64 = 10
};

union ParamValue {
    float f;
    int32_t i32;
    uint32_t u32;
    int16_t i16;
    uint16_t u16;
    int8_t i8;
    uint8_t u8;
};

struct Parameter {
    char name[16];
    ParamValue value;
    uint8_t type; // MAV_PARAM_TYPE
    bool exists;
};

#define MAX_PARAMETERS 100
static Parameter g_parameters[MAX_PARAMETERS];
static int g_paramCount = 0;

// Forward declaration
bool addParameter(const char* name, float value, uint8_t type);

void initParameters() {
    g_paramCount = 0;
    memset(g_parameters, 0, sizeof(g_parameters));

    // Add some default parameters
    addParameter("SYSID_THISMAV", 1.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("COMPID_MYGCS", 190.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("RC_SPEED", 490.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("THR_MIN", 130.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("THR_MID", 500.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("THR_MAX", 1000.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("ANGLE_MAX", 4500.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("ACRO_YAW_P", 4.5f, MAV_PARAM_TYPE_REAL32);
    addParameter("POS_XY_P", 1.0f, MAV_PARAM_TYPE_REAL32);
    addParameter("VEL_XY_P", 2.0f, MAV_PARAM_TYPE_REAL32);
}

bool addParameter(const char* name, float value, uint8_t type) {
    if (g_paramCount >= MAX_PARAMETERS) return false;

    Parameter& p = g_parameters[g_paramCount];
    strncpy(p.name, name, 15);
    p.name[15] = '\0';
    p.value.f = value;
    p.type = type;
    p.exists = true;

    g_paramCount++;
    return true;
}

int findParameterByName(const char* name) {
    for (int i = 0; i < g_paramCount; i++) {
        if (g_parameters[i].exists && strcmp(g_parameters[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

Parameter* getParameterByIndex(int index) {
    if (index < 0 || index >= g_paramCount) return nullptr;
    if (!g_parameters[index].exists) return nullptr;
    return &g_parameters[index];
}

Parameter* getParameterByName(const char* name) {
    int index = findParameterByName(name);
    if (index < 0) return nullptr;
    return &g_parameters[index];
}

bool setParameterValue(const char* name, float value, uint8_t type) {
    int index = findParameterByName(name);
    if (index < 0) return false;

    Parameter& p = g_parameters[index];

    // Convert value based on type
    switch (type) {
        case MAV_PARAM_TYPE_UINT8:
            p.value.u8 = (uint8_t)value;
            break;
        case MAV_PARAM_TYPE_INT8:
            p.value.i8 = (int8_t)value;
            break;
        case MAV_PARAM_TYPE_UINT16:
            p.value.u16 = (uint16_t)value;
            break;
        case MAV_PARAM_TYPE_INT16:
            p.value.i16 = (int16_t)value;
            break;
        case MAV_PARAM_TYPE_UINT32:
            p.value.u32 = (uint32_t)value;
            break;
        case MAV_PARAM_TYPE_INT32:
            p.value.i32 = (int32_t)value;
            break;
        case MAV_PARAM_TYPE_REAL32:
            p.value.f = value;
            break;
        default:
            return false;
    }

    p.type = type;
    return true;
}

//=============================================================================
// Parameter Request/Reply Functions
//=============================================================================

struct ParamRequestList {
    uint8_t targetSystem;
    uint8_t targetComponent;
};

struct ParamRequestRead {
    uint8_t targetSystem;
    uint8_t targetComponent;
    char paramID[16];
    int16_t paramIndex;
};

struct ParamSet {
    uint8_t targetSystem;
    uint8_t targetComponent;
    char paramID[16];
    float paramValue;
    uint8_t paramType;
};

struct ParamValueMsg {
    char paramID[16];
    float paramValue;
    uint8_t paramType;
    uint16_t paramCount;
    uint16_t paramIndex;
};

bool handleParamRequestList(const ParamRequestList& req) {
    // In real implementation, this would start streaming PARAM_VALUE messages
    // For testing, we just verify we can access all parameters
    return (g_paramCount > 0);
}

bool handleParamRequestRead(const ParamRequestRead& req, ParamValueMsg& reply) {
    Parameter* p = nullptr;

    if (req.paramIndex >= 0) {
        // Request by index
        p = getParameterByIndex(req.paramIndex);
        if (!p) return false;
    } else {
        // Request by name
        p = getParameterByName(req.paramID);
        if (!p) return false;
    }

    // Fill reply
    strncpy(reply.paramID, p->name, 16);
    reply.paramValue = p->value.f;
    reply.paramType = p->type;
    reply.paramCount = g_paramCount;
    reply.paramIndex = findParameterByName(p->name);

    return true;
}

bool handleParamSet(const ParamSet& req, ParamValueMsg& reply) {
    if (!setParameterValue(req.paramID, req.paramValue, req.paramType)) {
        return false;
    }

    // Send reply with updated value
    Parameter* p = getParameterByName(req.paramID);
    if (!p) return false;

    strncpy(reply.paramID, p->name, 16);
    reply.paramValue = p->value.f;
    reply.paramType = p->type;
    reply.paramCount = g_paramCount;
    reply.paramIndex = findParameterByName(p->name);

    return true;
}

//=============================================================================
// Tests
//=============================================================================

bool test_parameter_initialization() {
    initParameters();

    TEST_ASSERT(g_paramCount == 10, "Should have 10 parameters");

    Parameter* p = getParameterByName("SYSID_THISMAV");
    TEST_ASSERT(p != nullptr, "SYSID_THISMAV should exist");
    TEST_ASSERT(p->value.f == 1.0f, "SYSID_THISMAV value should be 1.0");

    TEST_PASS();
}

bool test_parameter_request_list() {
    initParameters();

    ParamRequestList req;
    req.targetSystem = 1;
    req.targetComponent = 1;

    bool success = handleParamRequestList(req);
    TEST_ASSERT(success, "Parameter request list should succeed");

    TEST_PASS();
}

bool test_parameter_request_read_by_index() {
    initParameters();

    ParamRequestRead req;
    req.targetSystem = 1;
    req.targetComponent = 1;
    req.paramIndex = 0;
    req.paramID[0] = '\0';

    ParamValueMsg reply;
    bool success = handleParamRequestRead(req, reply);

    TEST_ASSERT(success, "Read by index should succeed");
    TEST_ASSERT(strcmp(reply.paramID, "SYSID_THISMAV") == 0, "Should return first parameter");
    TEST_ASSERT(reply.paramCount == 10, "Should report total count");
    TEST_ASSERT(reply.paramIndex == 0, "Should be index 0");

    TEST_PASS();
}

bool test_parameter_request_read_by_name() {
    initParameters();

    ParamRequestRead req;
    req.targetSystem = 1;
    req.targetComponent = 1;
    req.paramIndex = -1; // Use name instead
    strncpy(req.paramID, "THR_MID", 16);

    ParamValueMsg reply;
    bool success = handleParamRequestRead(req, reply);

    TEST_ASSERT(success, "Read by name should succeed");
    TEST_ASSERT(strcmp(reply.paramID, "THR_MID") == 0, "Should return THR_MID");
    TEST_ASSERT(reply.paramValue == 500.0f, "THR_MID value should be 500.0");

    TEST_PASS();
}

bool test_parameter_request_read_invalid_index() {
    initParameters();

    ParamRequestRead req;
    req.targetSystem = 1;
    req.targetComponent = 1;
    req.paramIndex = 999; // Invalid index
    req.paramID[0] = '\0';

    ParamValueMsg reply;
    bool success = handleParamRequestRead(req, reply);

    TEST_ASSERT(!success, "Read with invalid index should fail");

    TEST_PASS();
}

bool test_parameter_request_read_invalid_name() {
    initParameters();

    ParamRequestRead req;
    req.targetSystem = 1;
    req.targetComponent = 1;
    req.paramIndex = -1;
    strncpy(req.paramID, "NONEXISTENT", 16);

    ParamValueMsg reply;
    bool success = handleParamRequestRead(req, reply);

    TEST_ASSERT(!success, "Read with invalid name should fail");

    TEST_PASS();
}

bool test_parameter_set_float() {
    initParameters();

    ParamSet req;
    req.targetSystem = 1;
    req.targetComponent = 1;
    strncpy(req.paramID, "ANGLE_MAX", 16);
    req.paramValue = 3000.0f;
    req.paramType = MAV_PARAM_TYPE_REAL32;

    ParamValueMsg reply;
    bool success = handleParamSet(req, reply);

    TEST_ASSERT(success, "Set parameter should succeed");
    TEST_ASSERT(reply.paramValue == 3000.0f, "Parameter value should be updated");

    // Verify the change
    Parameter* p = getParameterByName("ANGLE_MAX");
    TEST_ASSERT(p->value.f == 3000.0f, "Parameter should be changed in storage");

    TEST_PASS();
}

bool test_parameter_set_invalid_name() {
    initParameters();

    ParamSet req;
    req.targetSystem = 1;
    req.targetComponent = 1;
    strncpy(req.paramID, "INVALID_PARAM", 16);
    req.paramValue = 123.0f;
    req.paramType = MAV_PARAM_TYPE_REAL32;

    ParamValueMsg reply;
    bool success = handleParamSet(req, reply);

    TEST_ASSERT(!success, "Set invalid parameter should fail");

    TEST_PASS();
}

bool test_parameter_type_uint8() {
    initParameters();
    addParameter("TEST_U8", 0.0f, MAV_PARAM_TYPE_UINT8);

    ParamSet req;
    strncpy(req.paramID, "TEST_U8", 16);
    req.paramValue = 255.0f;
    req.paramType = MAV_PARAM_TYPE_UINT8;

    ParamValueMsg reply;
    bool success = handleParamSet(req, reply);
    TEST_ASSERT(success, "Set uint8 should succeed");

    Parameter* p = getParameterByName("TEST_U8");
    TEST_ASSERT(p->value.u8 == 255, "uint8 value should be 255");

    TEST_PASS();
}

bool test_parameter_type_int16() {
    initParameters();
    addParameter("TEST_I16", 0.0f, MAV_PARAM_TYPE_INT16);

    ParamSet req;
    strncpy(req.paramID, "TEST_I16", 16);
    req.paramValue = -32000.0f;
    req.paramType = MAV_PARAM_TYPE_INT16;

    ParamValueMsg reply;
    bool success = handleParamSet(req, reply);
    TEST_ASSERT(success, "Set int16 should succeed");

    Parameter* p = getParameterByName("TEST_I16");
    TEST_ASSERT(p->value.i16 == -32000, "int16 value should be -32000");

    TEST_PASS();
}

bool test_parameter_type_uint32() {
    initParameters();
    addParameter("TEST_U32", 0.0f, MAV_PARAM_TYPE_UINT32);

    ParamSet req;
    strncpy(req.paramID, "TEST_U32", 16);
    req.paramValue = 4000000000.0f;
    req.paramType = MAV_PARAM_TYPE_UINT32;

    ParamValueMsg reply;
    bool success = handleParamSet(req, reply);
    TEST_ASSERT(success, "Set uint32 should succeed");

    Parameter* p = getParameterByName("TEST_U32");
    TEST_ASSERT(p->value.u32 == 4000000000, "uint32 value should be correct");

    TEST_PASS();
}

bool test_parameter_count_accuracy() {
    initParameters();

    // Request each parameter and verify count is consistent
    for (int i = 0; i < g_paramCount; i++) {
        ParamRequestRead req;
        req.paramIndex = i;
        req.paramID[0] = '\0';

        ParamValueMsg reply;
        bool success = handleParamRequestRead(req, reply);
        TEST_ASSERT(success, "Read should succeed");
        TEST_ASSERT(reply.paramCount == 10, "Count should always be 10");
        TEST_ASSERT(reply.paramIndex == i, "Index should match");
    }

    TEST_PASS();
}

bool test_parameter_name_length() {
    initParameters();

    // Add parameter with maximum name length (16 chars including null)
    addParameter("LONGPARAMNAME15", 123.0f, MAV_PARAM_TYPE_REAL32);

    Parameter* p = getParameterByName("LONGPARAMNAME15");
    TEST_ASSERT(p != nullptr, "Long name parameter should exist");
    TEST_ASSERT(strlen(p->name) == 15, "Name should be 15 characters");

    TEST_PASS();
}

bool test_parameter_bounds() {
    initParameters();

    // Test that we can't add more than MAX_PARAMETERS
    int originalCount = g_paramCount;

    // Try to fill to capacity
    for (int i = originalCount; i < MAX_PARAMETERS; i++) {
        char name[16];
        snprintf(name, 16, "PARAM_%d", i);
        bool success = addParameter(name, (float)i, MAV_PARAM_TYPE_REAL32);
        TEST_ASSERT(success, "Should be able to add up to MAX_PARAMETERS");
    }

    // Try to add one more (should fail)
    bool success = addParameter("OVERFLOW", 0.0f, MAV_PARAM_TYPE_REAL32);
    TEST_ASSERT(!success, "Should fail when exceeding MAX_PARAMETERS");

    TEST_PASS();
}

bool test_multiple_parameter_updates() {
    initParameters();

    // Update multiple parameters in sequence
    const char* params[] = {"THR_MIN", "THR_MID", "THR_MAX"};
    float values[] = {100.0f, 450.0f, 900.0f};

    for (int i = 0; i < 3; i++) {
        ParamSet req;
        strncpy(req.paramID, params[i], 16);
        req.paramValue = values[i];
        req.paramType = MAV_PARAM_TYPE_REAL32;

        ParamValueMsg reply;
        bool success = handleParamSet(req, reply);
        TEST_ASSERT(success, "Parameter update should succeed");
        TEST_ASSERT(reply.paramValue == values[i], "Reply value should match");
    }

    // Verify all updates
    for (int i = 0; i < 3; i++) {
        Parameter* p = getParameterByName(params[i]);
        TEST_ASSERT(p->value.f == values[i], "Stored value should match");
    }

    TEST_PASS();
}

bool test_parameter_streaming_order() {
    initParameters();

    // Verify parameters are returned in order
    for (int i = 0; i < g_paramCount; i++) {
        ParamRequestRead req;
        req.paramIndex = i;
        req.paramID[0] = '\0';

        ParamValueMsg reply;
        bool success = handleParamRequestRead(req, reply);
        TEST_ASSERT(success, "Read should succeed");
        TEST_ASSERT(reply.paramIndex == (uint16_t)i, "Index should be sequential");
    }

    TEST_PASS();
}

//=============================================================================
// Test Runner
//=============================================================================

struct Test {
    const char* name;
    bool (*func)();
};

int main() {
    printf("\n=== MAVLink Parameters Test Suite ===\n\n");

    Test tests[] = {
        {"Parameter Initialization", test_parameter_initialization},
        {"Parameter Request List", test_parameter_request_list},
        {"Parameter Request Read by Index", test_parameter_request_read_by_index},
        {"Parameter Request Read by Name", test_parameter_request_read_by_name},
        {"Parameter Request Read Invalid Index", test_parameter_request_read_invalid_index},
        {"Parameter Request Read Invalid Name", test_parameter_request_read_invalid_name},
        {"Parameter Set Float", test_parameter_set_float},
        {"Parameter Set Invalid Name", test_parameter_set_invalid_name},
        {"Parameter Type uint8", test_parameter_type_uint8},
        {"Parameter Type int16", test_parameter_type_int16},
        {"Parameter Type uint32", test_parameter_type_uint32},
        {"Parameter Count Accuracy", test_parameter_count_accuracy},
        {"Parameter Name Length", test_parameter_name_length},
        {"Parameter Bounds", test_parameter_bounds},
        {"Multiple Parameter Updates", test_multiple_parameter_updates},
        {"Parameter Streaming Order", test_parameter_streaming_order}
    };

    int numTests = sizeof(tests) / sizeof(tests[0]);
    int passed = 0;
    int failed = 0;

    for (int i = 0; i < numTests; i++) {
        printf("Running: %s\n", tests[i].name);
        if (tests[i].func()) {
            printf("  PASS\n");
            passed++;
        } else {
            failed++;
        }
    }

    printf("\n=== Test Summary ===\n");
    printf("Total:  %d\n", numTests);
    printf("Passed: %d\n", passed);
    printf("Failed: %d\n", failed);
    printf("Success Rate: %.1f%%\n", (100.0 * passed) / numTests);

    if (failed == 0) {
        printf("\n✓ All tests passed!\n\n");
        return 0;
    } else {
        printf("\n✗ Some tests failed!\n\n");
        return 1;
    }
}
