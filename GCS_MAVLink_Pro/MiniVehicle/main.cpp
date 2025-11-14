/*
 * main.cpp
 *
 * Entry point for MiniVehicle
 * This is the standard ArduPilot vehicle entry point pattern
 */

#include <AP_HAL/AP_HAL.h>
#include "MiniVehicle.h"

// ========================================
// HAL INSTANCE
// ========================================

/*
 * The HAL (Hardware Abstraction Layer) provides platform-independent
 * access to hardware features like:
 * - Serial ports
 * - Timers
 * - GPIO
 * - I2C/SPI
 * - Storage
 * etc.
 *
 * This is provided by the board-specific HAL implementation
 */
extern const AP_HAL::HAL& hal;

// ========================================
// FUNCTION DECLARATIONS
// ========================================

/*
 * These functions are called by the HAL:
 * - setup() is called once at boot
 * - loop() is called repeatedly
 */
void setup();
void loop();

// ========================================
// SETUP - Called once at boot
// ========================================

void setup()
{
    /*
     * This is the first function called after boot
     *
     * It delegates to the vehicle's setup() function
     * which handles all initialization:
     * - HAL initialization
     * - Serial ports
     * - GCS (INTEGRATION STEP 4: gcs().init(), setup_console(), setup_uarts())
     * - AHRS
     * - Sensors
     * - etc.
     */

    minivehicle.setup();
}

// ========================================
// LOOP - Called repeatedly
// ========================================

void loop()
{
    /*
     * This is called as fast as possible after setup()
     *
     * It delegates to the vehicle's loop() function
     * which handles:
     * - Fast loop (high-frequency control)
     * - Slow loop (low-frequency updates)
     * - GCS updates (INTEGRATION STEP 3: update_receive(), update_send())
     * - Everything else
     *
     * The vehicle's loop() function manages timing internally
     * to ensure different tasks run at appropriate rates
     */

    minivehicle.loop();
}

// ========================================
// HAL MAIN MACRO
// ========================================

/*
 * This macro provides the actual main() function
 * It's board-specific and handles:
 * - Hardware initialization
 * - Calling setup() once
 * - Calling loop() repeatedly
 * - Scheduler management
 * - Watchdog handling
 * - etc.
 *
 * Example expansions:
 *
 * For Linux:
 *   int main(int argc, char* argv[]) {
 *       hal.run(argc, argv, setup, loop);
 *   }
 *
 * For STM32:
 *   int main(void) {
 *       hw_init();
 *       setup();
 *       while(1) {
 *           loop();
 *           scheduler.run();
 *       }
 *   }
 *
 * For ChibiOS:
 *   int main(void) {
 *       halInit();
 *       chSysInit();
 *       setup();
 *       while(true) {
 *           loop();
 *       }
 *   }
 */
AP_HAL_MAIN();

/*
 * ============================================
 * That's it!
 * ============================================
 *
 * This simple file is all that's needed for the entry point.
 * Everything else happens in:
 * - MiniVehicle.cpp: Main vehicle logic
 * - system.cpp: System-level functions
 * - GCS_MiniVehicle.cpp: GCS manager
 * - GCS_MAVLink_MiniVehicle.cpp: MAVLink channel
 *
 * The beauty of this architecture is that the entry point
 * is trivial - all the complexity is properly organized
 * in the appropriate files.
 */
