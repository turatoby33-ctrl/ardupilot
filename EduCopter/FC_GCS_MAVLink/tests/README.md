# EduCopter GCS_MAVLink Test Suite

This directory contains comprehensive unit tests for the EduCopter GCS_MAVLink implementation.

## Test Files

### test_routing.cpp
Tests the MAVLink message routing system including:
- Route learning from heartbeat and other messages
- Route table management (add, remove, update)
- Route lookup by system ID, component ID, and MAV_TYPE
- Stale route cleanup and timeout handling
- Channel blocking functionality
- Route table capacity limits
- Wildcard component lookups

**Tests:** 14 test cases covering all routing functionality

### test_heartbeat.cpp
Tests heartbeat message generation, parsing, and connection monitoring:
- Heartbeat message packing and unpacking
- Vehicle type identification
- System state reporting (uninit, boot, standby, active, critical, emergency)
- Base mode flags (armed, manual, auto, guided, stabilize, etc.)
- Custom flight mode handling
- Component identification (autopilot, computer, camera, etc.)
- Connection timeout detection
- Heartbeat timing and intervals
- GCS heartbeat handling
- Armed/disarmed state detection
- Flight mode transitions

**Tests:** 15 test cases covering all heartbeat functionality

## Building the Tests

### Prerequisites
- GCC or Clang compiler with C++11 support
- MAVLink v2.0 headers installed
- EduCopter GCS_MAVLink source files

### Compile Commands

#### Test Routing
```bash
cd /home/user/ardupilot/EduCopter/FC_GCS_MAVLink/tests

g++ -std=c++11 -o test_routing test_routing.cpp \
    ../MAVLink_routing.cpp \
    -I.. \
    -I/path/to/mavlink/include \
    -DEDUCOPTER_GCS_ENABLED=1

./test_routing
```

#### Test Heartbeat
```bash
g++ -std=c++11 -o test_heartbeat test_heartbeat.cpp \
    -I.. \
    -I/path/to/mavlink/include \
    -DEDUCOPTER_GCS_ENABLED=1

./test_heartbeat
```

### Expected Output

Both test suites should output:
```
=== MAVLink [Routing/Heartbeat] Test Suite ===

PASS: test_name_1
PASS: test_name_2
...

=== Test Summary ===
Total:  XX
Passed: XX
Failed: 0
Success Rate: 100.0%

✓ All tests passed!
```

## Running All Tests

Create a simple script to run all tests:

```bash
#!/bin/bash
# run_all_tests.sh

echo "Running all EduCopter GCS_MAVLink tests..."
echo ""

./test_routing
ROUTING_RESULT=$?

./test_heartbeat
HEARTBEAT_RESULT=$?

echo ""
echo "==================================="
if [ $ROUTING_RESULT -eq 0 ] && [ $HEARTBEAT_RESULT -eq 0 ]; then
    echo "✓ ALL TESTS PASSED"
    exit 0
else
    echo "✗ SOME TESTS FAILED"
    exit 1
fi
```

## Test Coverage

### Routing Tests Cover:
- ✅ Router initialization and cleanup
- ✅ Single and multiple route management
- ✅ Route learning from various message types
- ✅ Manual route addition
- ✅ Route updates when systems change channels
- ✅ MAV_TYPE-based route lookup
- ✅ Wildcard component ID matching
- ✅ Route aging and refresh
- ✅ Stale route removal (timeout)
- ✅ Channel blocking/unblocking
- ✅ Route table capacity limits
- ✅ Clear all routes functionality

### Heartbeat Tests Cover:
- ✅ Message packing and encoding
- ✅ Message unpacking and decoding
- ✅ All vehicle types (quadrotor, helicopter, fixed wing, etc.)
- ✅ All system states (uninit, boot, standby, active, critical, emergency, poweroff)
- ✅ All base mode flags (armed, manual, auto, guided, stabilize, test, HIL)
- ✅ Custom flight modes
- ✅ Multiple component heartbeats
- ✅ Heartbeat timing intervals
- ✅ Connection timeout detection
- ✅ MAVLink version field
- ✅ GCS heartbeat handling
- ✅ Armed/disarmed state detection
- ✅ Flight mode detection
- ✅ Connection quality assessment
- ✅ Emergency state handling

## Adding New Tests

To add new test cases:

1. Create a new test function following the pattern:
```cpp
bool test_new_feature() {
    // Setup

    // Execute test

    // Verify with TEST_ASSERT
    TEST_ASSERT(condition, "error message");

    // Success
    TEST_PASS();
}
```

2. Add the test to the test list in `runAllTests()`:
```cpp
Test tests[] = {
    // ... existing tests ...
    {"New Feature", test_new_feature}
};
```

3. Rebuild and run the test suite

## Mock Functions

The test files include mock implementations for:
- `millis()` - System millisecond timer
- `micros()` - System microsecond timer
- `millis16()` - 16-bit millisecond timer
- `getBaseMode()` - Vehicle base mode flags
- `getSystemStatus()` - Vehicle system status

These can be adjusted in tests to simulate different conditions.

## Continuous Integration

These tests can be integrated into a CI/CD pipeline:

```yaml
# .github/workflows/test.yml
name: GCS_MAVLink Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Install MAVLink
        run: |
          git clone https://github.com/mavlink/mavlink.git
      - name: Build Tests
        run: |
          cd EduCopter/FC_GCS_MAVLink/tests
          ./build_tests.sh
      - name: Run Tests
        run: |
          cd EduCopter/FC_GCS_MAVLink/tests
          ./run_all_tests.sh
```

## Test Results Archive

Test results can be saved for tracking:

```bash
./test_routing > test_results_routing_$(date +%Y%m%d_%H%M%S).log
./test_heartbeat > test_results_heartbeat_$(date +%Y%m%d_%H%M%S).log
```

## Known Issues

None currently. All tests should pass on a properly configured system.

## Contributing

When contributing new features to GCS_MAVLink:
1. Write tests FIRST (TDD approach)
2. Ensure all existing tests still pass
3. Add new tests to cover new functionality
4. Update this README with new test descriptions

## Support

For questions or issues with the tests:
- Check that MAVLink headers are properly installed
- Verify compiler supports C++11
- Ensure all GCS_MAVLink source files are present
- Check for proper namespace usage (EduCopter::GCS)

---

**Total Test Count:** 29 tests (14 routing + 15 heartbeat)
**Expected Success Rate:** 100%
**Execution Time:** < 1 second for all tests
