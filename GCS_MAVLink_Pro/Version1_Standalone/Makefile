# Makefile for Minimal Standalone GCS (Version 1)

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -g
TARGET = test_heartbeat
SOURCES = test_heartbeat.cpp minimal_GCS.cpp minimal_GCS_Common.cpp
OBJECTS = $(SOURCES:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^
	@echo ""
	@echo "============================================"
	@echo "Build successful!"
	@echo "Run with: ./$(TARGET)"
	@echo "============================================"

%.o: %.cpp minimal_GCS.h
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(OBJECTS) $(TARGET)
	@echo "Cleaned build artifacts"

run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
