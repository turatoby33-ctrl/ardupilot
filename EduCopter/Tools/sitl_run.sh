#!/bin/bash

# EduCopter SITL launch script

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EDUCOPTER_ROOT="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$EDUCOPTER_ROOT/build"

echo "========================================="
echo " EduCopter SITL Launcher"
echo "========================================="
echo ""

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo "Build directory not found. Running build..."
    echo ""

    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"

    cmake ..
    make -j$(nproc)

    if [ $? -ne 0 ]; then
        echo ""
        echo "ERROR: Build failed!"
        exit 1
    fi

    echo ""
    echo "Build complete!"
    echo ""
fi

# Check if executable exists
EXECUTABLE="$BUILD_DIR/educopter_sitl"

if [ ! -f "$EXECUTABLE" ]; then
    echo "ERROR: Executable not found at $EXECUTABLE"
    echo "Please run: cd $BUILD_DIR && cmake .. && make"
    exit 1
fi

# Change to build directory to run (so logs are created there)
cd "$BUILD_DIR"

echo "Starting EduCopter SITL..."
echo "Log files will be created in: $BUILD_DIR"
echo ""
echo "Controls (simulated):"
echo "  - Throttle: Automated for testing"
echo "  - Arming: Automatic after 3 seconds"
echo "  - Press Ctrl+C to stop"
echo ""

# Run SITL
"$EXECUTABLE" "$@"

echo ""
echo "SITL stopped."
