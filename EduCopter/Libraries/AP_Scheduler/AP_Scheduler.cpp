#include "AP_Scheduler.h"
#include "../AP_HAL/AP_HAL.h"
#include <algorithm>
#include <cstdio>

extern AP_HAL* hal;

AP_Scheduler::AP_Scheduler() :
    loop_rate_hz(400),  // Default 400Hz main loop
    last_loop_time_us(0)
{
    perf.max_time_us = 0;
    perf.avg_time_us = 0;
    perf.overrun_count = 0;
}

void AP_Scheduler::init() {
    printf("AP_Scheduler: Initialized with %zu tasks\n", tasks.size());
}

void AP_Scheduler::register_task(std::function<void(void)> func, uint32_t rate_hz, uint32_t max_time_us) {
    AP_Scheduler_Task task;
    task.function = func;
    task.interval_us = 1000000 / rate_hz;
    task.max_time_us = max_time_us;
    task.last_run_us = 0;
    tasks.push_back(task);
}

void AP_Scheduler::run(uint64_t time_available_us) {
    uint64_t now_us = hal->micros64();
    uint64_t loop_start_us = now_us;

    // Run all tasks that are due
    for (auto& task : tasks) {
        // Check if task is due
        if (now_us - task.last_run_us >= task.interval_us) {
            uint64_t task_start_us = hal->micros64();

            // Run the task
            task.function();

            uint64_t task_end_us = hal->micros64();
            uint32_t task_time_us = task_end_us - task_start_us;

            // Update task timing
            task.last_run_us = task_start_us;

            // Check for overrun
            if (task_time_us > task.max_time_us) {
                perf.overrun_count++;
            }

            // Update performance stats
            if (task_time_us > perf.max_time_us) {
                perf.max_time_us = task_time_us;
            }

            now_us = task_end_us;

            // Check if we've used up our time
            if (now_us - loop_start_us >= time_available_us) {
                break;
            }
        }
    }

    // Calculate loop timing
    uint64_t loop_end_us = hal->micros64();
    uint32_t loop_time_us = loop_end_us - loop_start_us;

    // Update average (simple moving average)
    perf.avg_time_us = (perf.avg_time_us * 9 + loop_time_us) / 10;
}
