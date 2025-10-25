#include "EduCopter.h"
#include "Libraries/AP_HAL_SITL/AP_HAL_SITL.h"
#include <signal.h>
#include <cstdio>
#include <thread>
#include <chrono>

// Global HAL instance
AP_HAL* hal = nullptr;
AP_HAL_SITL* sitl_hal = nullptr;

// Signal handling for clean shutdown
volatile bool should_exit = false;

void signal_handler(int sig) {
    printf("\nReceived signal %d, shutting down...\n", sig);
    should_exit = true;
}

// RC input thread (simulates RC transmitter)
void rc_input_thread() {
    printf("RC Input: Starting RC input thread\n");
    printf("RC Input: Controls:\n");
    printf("  Throttle: Starts at 0%%, will auto-increase for testing\n");
    printf("  Roll/Pitch/Yaw: Centered\n\n");

    uint32_t loop_count = 0;

    while (!should_exit) {
        // Set default RC inputs (centered, throttle low)
        sitl_hal->set_rc_input(0, 1500);  // Roll
        sitl_hal->set_rc_input(1, 1500);  // Pitch
        sitl_hal->set_rc_input(2, 1000);  // Throttle (start low)
        sitl_hal->set_rc_input(3, 1500);  // Yaw

        // After 2 seconds, increase throttle for arming
        if (loop_count > 200) {
            sitl_hal->set_rc_input(2, 1100);  // Throttle slightly up for flight
        }

        // After 3 seconds, move yaw right to arm
        if (loop_count > 300 && loop_count < 350) {
            sitl_hal->set_rc_input(3, 1900);  // Yaw right to arm
        } else if (loop_count >= 350) {
            sitl_hal->set_rc_input(3, 1500);  // Center yaw
        }

        // After arming (5 seconds), increase throttle
        if (loop_count > 500) {
            sitl_hal->set_rc_input(2, 1600);  // 60% throttle
        }

        // At 10 seconds, add some roll input
        if (loop_count > 1000 && loop_count < 1500) {
            sitl_hal->set_rc_input(0, 1600);  // Roll right
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        loop_count++;
    }
}

// Physics update thread
void physics_thread() {
    printf("Physics: Starting physics simulation thread\n");

    auto last_time = std::chrono::steady_clock::now();

    while (!should_exit) {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        last_time = now;

        // Limit dt to prevent instability
        if (dt > 0.01f) {
            dt = 0.01f;
        }

        // Update physics simulation
        if (sitl_hal) {
            sitl_hal->get_physics().update(dt);
        }

        // Run at ~1000Hz
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }
}

int main(int argc, char* argv[]) {
    printf("\n");
    printf("╔════════════════════════════════════════════╗\n");
    printf("║  EduCopter SITL - Software In The Loop   ║\n");
    printf("║  Educational Quadcopter Flight Controller ║\n");
    printf("║  Version 1.0                              ║\n");
    printf("╚════════════════════════════════════════════╝\n");
    printf("\n");

    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Create SITL HAL
    sitl_hal = new AP_HAL_SITL();
    hal = sitl_hal;

    printf("SITL: HAL created\n");

    // Initialize copter
    copter.init();

    printf("\n");
    printf("SITL: Starting simulation threads...\n");

    // Start physics thread
    std::thread physics_t(physics_thread);

    // Start RC input thread
    std::thread rc_t(rc_input_thread);

    printf("SITL: Simulation started\n");
    printf("SITL: Press Ctrl+C to stop\n\n");

    // Main loop
    uint32_t frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (!should_exit) {
        copter.loop();

        frame_count++;

        // Print status every 5 seconds
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<float>(now - start_time).count();

        if (elapsed >= 5.0f) {
            float loop_rate = frame_count / elapsed;

            const Vector3f& pos = sitl_hal->get_physics().get_position();
            const Vector3f& vel = sitl_hal->get_physics().get_velocity();

            printf("Status: Loop rate: %.1f Hz | Pos: [%.2f, %.2f, %.2f] m | "
                   "Vel: [%.2f, %.2f, %.2f] m/s | Armed: %s\n",
                   loop_rate, pos.x, pos.y, -pos.z,
                   vel.x, vel.y, vel.z,
                   copter.is_armed() ? "YES" : "NO");

            start_time = now;
            frame_count = 0;
        }
    }

    printf("\nSITL: Shutting down...\n");

    // Stop threads
    rc_t.join();
    physics_t.join();

    // Cleanup
    delete sitl_hal;
    hal = nullptr;

    printf("SITL: Shutdown complete\n");

    return 0;
}
