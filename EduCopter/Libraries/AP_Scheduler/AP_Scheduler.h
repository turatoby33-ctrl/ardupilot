#pragma once

#include <stdint.h>
#include <functional>
#include <vector>

// Task structure
struct AP_Scheduler_Task {
    std::function<void(void)> function;  // Function to call
    uint32_t interval_us;                // Interval in microseconds
    uint32_t max_time_us;                // Maximum expected runtime
    uint32_t last_run_us;                // Last execution time
};

class AP_Scheduler {
public:
    AP_Scheduler();

    // Initialize scheduler
    void init();

    // Add a task to the scheduler
    void register_task(std::function<void(void)> func, uint32_t rate_hz, uint32_t max_time_us = 1000);

    // Run the scheduler (call this in main loop)
    void run(uint64_t time_available_us);

    // Get timing information
    uint32_t get_loop_rate_hz() const { return loop_rate_hz; }
    uint32_t get_loop_period_us() const { return 1000000 / loop_rate_hz; }

    // Performance monitoring
    struct {
        uint32_t max_time_us;
        uint32_t avg_time_us;
        uint32_t overrun_count;
    } perf;

private:
    std::vector<AP_Scheduler_Task> tasks;
    uint32_t loop_rate_hz;
    uint64_t last_loop_time_us;
};
