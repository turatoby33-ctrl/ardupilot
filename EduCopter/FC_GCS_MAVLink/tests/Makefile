# Makefile for EduCopter GCS_MAVLink Test Suite
# Builds and runs comprehensive unit tests

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -I.. -DEDUCOPTER_GCS_ENABLED=1
LDFLAGS =

# MAVLink include path (adjust as needed)
MAVLINK_INCLUDE = -I/usr/local/include/mavlink/v2.0

# Source directories
SRC_DIR = ..
TEST_DIR = .

# Test executables
TEST_ROUTING = test_routing
TEST_HEARTBEAT = test_heartbeat

# Source files
ROUTING_SRC = $(TEST_DIR)/test_routing.cpp $(SRC_DIR)/MAVLink_routing.cpp
HEARTBEAT_SRC = $(TEST_DIR)/test_heartbeat.cpp

# All test targets
TESTS = $(TEST_ROUTING) $(TEST_HEARTBEAT)

.PHONY: all clean test help

# Default target
all: $(TESTS)
	@echo ""
	@echo "==================================="
	@echo "All tests built successfully!"
	@echo "Run 'make test' to execute tests"
	@echo "==================================="

# Build routing test
$(TEST_ROUTING): $(ROUTING_SRC)
	@echo "Building routing tests..."
	$(CXX) $(CXXFLAGS) $(MAVLINK_INCLUDE) -o $@ $^ $(LDFLAGS)

# Build heartbeat test
$(TEST_HEARTBEAT): $(HEARTBEAT_SRC)
	@echo "Building heartbeat tests..."
	$(CXX) $(CXXFLAGS) $(MAVLINK_INCLUDE) -o $@ $^ $(LDFLAGS)

# Run all tests
test: $(TESTS)
	@echo ""
	@echo "==================================="
	@echo "Running all EduCopter GCS_MAVLink tests..."
	@echo "==================================="
	@echo ""
	@./$(TEST_ROUTING) || exit 1
	@echo ""
	@./$(TEST_HEARTBEAT) || exit 1
	@echo ""
	@echo "==================================="
	@echo "✓ ALL TESTS PASSED"
	@echo "==================================="
	@echo ""

# Run specific test
test-routing: $(TEST_ROUTING)
	@echo "Running routing tests..."
	@./$(TEST_ROUTING)

test-heartbeat: $(TEST_HEARTBEAT)
	@echo "Running heartbeat tests..."
	@./$(TEST_HEARTBEAT)

# Clean build artifacts
clean:
	@echo "Cleaning test binaries..."
	@rm -f $(TESTS)
	@rm -f *.o
	@echo "Clean complete."

# Help target
help:
	@echo "EduCopter GCS_MAVLink Test Suite"
	@echo ""
	@echo "Available targets:"
	@echo "  make all              - Build all tests"
	@echo "  make test             - Build and run all tests"
	@echo "  make test-routing     - Run routing tests only"
	@echo "  make test-heartbeat   - Run heartbeat tests only"
	@echo "  make clean            - Remove build artifacts"
	@echo "  make help             - Show this help"
	@echo ""
	@echo "Tests:"
	@echo "  test_routing    - 14 tests for MAVLink routing"
	@echo "  test_heartbeat  - 15 tests for heartbeat handling"
	@echo ""
	@echo "Total: 29 comprehensive unit tests"
