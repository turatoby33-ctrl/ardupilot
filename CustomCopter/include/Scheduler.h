#pragma once

#include <cstdint>
#include <functional>
#include <vector>

namespace CustomCopter {

// ============================================================================
// Task Structure
// ============================================================================
struct Task {
    std::function<void()> func;     // Task function to call
    const char* name;                // Task name (for debugging)
    uint32_t rate_hz;                // Execution rate in Hz
    uint32_t max_time_us;            // Maximum execution time (microseconds)
    uint8_t priority;                // Task priority (0 = highest)

    // Runtime state
    uint64_t last_run_us;            // Last execution time
    uint32_t interval_us;            // Interval between executions
    uint32_t avg_time_us;            // Average execution time
    uint32_t max_time_recorded_us;   // Recorded max execution time
    uint32_t run_count;              // Number of times executed
};

// ============================================================================
// Scheduler Class
// Based on ArduPilot's AP_Scheduler
// ============================================================================
class Scheduler {
public:
    Scheduler();
    ~Scheduler() = default;

    // Initialize scheduler
    void init();

    // Main loop tick - call this every loop iteration
    void tick();

    // Add a task to the scheduler
    // func: Task function to execute
    // name: Task name (for debugging/logging)
    // rate_hz: Execution rate in Hz (0 = every loop)
    // max_time_us: Expected maximum execution time
    // priority: Task priority (0 = highest, higher numbers = lower priority)
    void add_task(std::function<void()> func, const char* name,
                  uint32_t rate_hz, uint32_t max_time_us, uint8_t priority);

    // Set the main loop rate (typically 400 Hz)
    void set_loop_rate_hz(uint32_t rate_hz);

    // Get the main loop rate
    uint32_t get_loop_rate_hz() const { return loop_rate_hz_; }

    // Get the last loop time in seconds
    float get_last_loop_time_s() const { return last_loop_time_us_ * 1e-6f; }

    // Get the current time in microseconds
    uint64_t micros() const;

    // Get the current time in milliseconds
    uint64_t millis() const;

    // Delay functions
    void delay_microseconds(uint32_t us);
    void delay_milliseconds(uint32_t ms);

    // Get scheduler statistics
    void print_stats() const;

    // Check if we're on time with the main loop
    bool is_on_time() const { return loop_overrun_count_ == 0; }

    // Get loop overrun count
    uint32_t get_loop_overrun_count() const { return loop_overrun_count_; }

private:
    // Run scheduled tasks
    void run_tasks();

    // Check if a task should run
    bool should_run_task(const Task& task) const;

    // Update task timing statistics
    void update_task_stats(Task& task, uint32_t elapsed_us);

    std::vector<Task> tasks_;

    uint32_t loop_rate_hz_;          // Main loop rate (e.g., 400 Hz)
    uint64_t last_loop_start_us_;    // Last loop start time
    uint32_t last_loop_time_us_;     // Last loop duration
    uint32_t target_loop_time_us_;   // Target loop time
    uint32_t loop_overrun_count_;    // Number of loop overruns

    bool initialized_;
};

} // namespace CustomCopter
