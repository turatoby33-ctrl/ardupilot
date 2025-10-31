#include "Scheduler.h"
#include <algorithm>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

namespace CustomCopter {

Scheduler::Scheduler()
    : loop_rate_hz_(400)
    , last_loop_start_us_(0)
    , last_loop_time_us_(0)
    , target_loop_time_us_(2500)  // 400 Hz = 2500 us
    , loop_overrun_count_(0)
    , initialized_(false)
{
}

void Scheduler::init() {
    last_loop_start_us_ = micros();
    target_loop_time_us_ = 1000000 / loop_rate_hz_;
    initialized_ = true;
}

void Scheduler::set_loop_rate_hz(uint32_t rate_hz) {
    loop_rate_hz_ = rate_hz;
    target_loop_time_us_ = 1000000 / loop_rate_hz_;
}

void Scheduler::tick() {
    if (!initialized_) {
        init();
    }

    // Get current time
    uint64_t now_us = micros();

    // Calculate last loop time
    if (last_loop_start_us_ > 0) {
        last_loop_time_us_ = now_us - last_loop_start_us_;

        // Check for loop overrun (took longer than target)
        if (last_loop_time_us_ > target_loop_time_us_) {
            loop_overrun_count_++;
        }
    }

    // Update loop start time
    last_loop_start_us_ = now_us;

    // Run scheduled tasks
    run_tasks();
}

void Scheduler::add_task(std::function<void()> func, const char* name,
                         uint32_t rate_hz, uint32_t max_time_us,
                         uint8_t priority) {
    Task task;
    task.func = func;
    task.name = name;
    task.rate_hz = rate_hz;
    task.max_time_us = max_time_us;
    task.priority = priority;
    task.last_run_us = 0;
    task.interval_us = (rate_hz > 0) ? (1000000 / rate_hz) : 0;
    task.avg_time_us = 0;
    task.max_time_recorded_us = 0;
    task.run_count = 0;

    tasks_.push_back(task);

    // Sort tasks by priority (lower number = higher priority)
    std::sort(tasks_.begin(), tasks_.end(),
              [](const Task& a, const Task& b) {
                  return a.priority < b.priority;
              });
}

void Scheduler::run_tasks() {
    for (auto& task : tasks_) {
        // Check if task should run this iteration
        if (should_run_task(task)) {
            // Record start time
            uint64_t start_us = micros();

            // Execute task
            task.func();

            // Update timing statistics
            uint64_t end_us = micros();
            uint32_t elapsed_us = end_us - start_us;
            update_task_stats(task, elapsed_us);

            // Update last run time
            task.last_run_us = start_us;
            task.run_count++;
        }
    }
}

bool Scheduler::should_run_task(const Task& task) const {
    // If rate is 0, run every loop (FAST_TASK)
    if (task.rate_hz == 0) {
        return true;
    }

    // Check if enough time has elapsed since last run
    uint64_t now_us = micros();
    if (task.last_run_us == 0) {
        return true;  // First run
    }

    uint64_t elapsed_us = now_us - task.last_run_us;
    return elapsed_us >= task.interval_us;
}

void Scheduler::update_task_stats(Task& task, uint32_t elapsed_us) {
    // Update average execution time (exponential moving average)
    if (task.avg_time_us == 0) {
        task.avg_time_us = elapsed_us;
    } else {
        // Alpha = 0.1 (10% weight to new value)
        task.avg_time_us = (task.avg_time_us * 9 + elapsed_us) / 10;
    }

    // Update max execution time
    if (elapsed_us > task.max_time_recorded_us) {
        task.max_time_recorded_us = elapsed_us;
    }

    // Warn if task exceeded expected max time
    if (elapsed_us > task.max_time_us) {
        std::cerr << "WARNING: Task '" << task.name
                  << "' took " << elapsed_us << " us (max: "
                  << task.max_time_us << " us)" << std::endl;
    }
}

uint64_t Scheduler::micros() const {
    auto now = std::chrono::steady_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

uint64_t Scheduler::millis() const {
    return micros() / 1000;
}

void Scheduler::delay_microseconds(uint32_t us) {
    std::this_thread::sleep_for(std::chrono::microseconds(us));
}

void Scheduler::delay_milliseconds(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void Scheduler::print_stats() const {
    std::cout << "\n=== Scheduler Statistics ===" << std::endl;
    std::cout << "Loop rate: " << loop_rate_hz_ << " Hz ("
              << target_loop_time_us_ << " us target)" << std::endl;
    std::cout << "Last loop time: " << last_loop_time_us_ << " us" << std::endl;
    std::cout << "Loop overruns: " << loop_overrun_count_ << std::endl;
    std::cout << "\nTask Statistics:" << std::endl;
    std::cout << std::setw(20) << "Name"
              << std::setw(10) << "Rate(Hz)"
              << std::setw(10) << "Priority"
              << std::setw(12) << "Runs"
              << std::setw(12) << "Avg(us)"
              << std::setw(12) << "Max(us)" << std::endl;
    std::cout << std::string(76, '-') << std::endl;

    for (const auto& task : tasks_) {
        std::cout << std::setw(20) << task.name
                  << std::setw(10) << (task.rate_hz == 0 ? "FAST" : std::to_string(task.rate_hz))
                  << std::setw(10) << static_cast<int>(task.priority)
                  << std::setw(12) << task.run_count
                  << std::setw(12) << task.avg_time_us
                  << std::setw(12) << task.max_time_recorded_us << std::endl;
    }
    std::cout << std::endl;
}

} // namespace CustomCopter
