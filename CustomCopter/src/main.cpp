#include "Copter.h"
#include <iostream>
#include <csignal>
#include <atomic>

// Signal handler for clean shutdown
std::atomic<bool> running(true);

void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "\nShutting down..." << std::endl;
        running = false;
    }
}

int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    // Setup signal handlers for clean shutdown
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "========================================" << std::endl;
    std::cout << "  CustomCopter Flight Controller v1.0  " << std::endl;
    std::cout << "  Based on ArduPilot/ArduCopter        " << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Create copter instance
    CustomCopter::Copter copter;

    // Initialize
    if (!copter.init()) {
        std::cerr << "FATAL: Initialization failed" << std::endl;
        return 1;
    }

    std::cout << std::endl;
    std::cout << "System initialized successfully" << std::endl;
    std::cout << "Press Ctrl+C to exit" << std::endl;
    std::cout << std::endl;

    // Run main loop (this will run until interrupted)
    // TODO: For testing, we'll run for a limited time
    // copter.loop();  // Infinite loop

    // For now, run for 10 seconds and show statistics
    std::cout << "Running for 10 seconds..." << std::endl;

    auto* scheduler = copter.get_scheduler();
    uint64_t start_time = scheduler->millis();
    uint64_t print_time = start_time;

    while (running && (scheduler->millis() - start_time) < 10000) {
        scheduler->tick();
        scheduler->delay_microseconds(100);

        // Print status every second
        if (scheduler->millis() - print_time > 1000) {
            std::cout << "Time: " << (scheduler->millis() - start_time) / 1000
                      << "s | Mode: " << copter.get_current_mode()->name()
                      << " | Armed: " << (copter.is_armed() ? "YES" : "NO")
                      << " | Healthy: " << (copter.is_healthy() ? "YES" : "NO")
                      << std::endl;
            print_time = scheduler->millis();
        }
    }

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Scheduler Statistics" << std::endl;
    std::cout << "========================================" << std::endl;
    scheduler->print_stats();

    std::cout << std::endl;
    std::cout << "Shutdown complete" << std::endl;

    return 0;
}
