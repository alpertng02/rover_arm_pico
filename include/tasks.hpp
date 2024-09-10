/**
 * @file tasks.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Header file for FreeRTOS tasks.
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef TASKS_HPP
#define TASKS_HPP

#include <FreeRTOS.h>
#include <task.h>

#include <etl/array.h>

namespace freertos {
namespace task {

inline TaskHandle_t microRosTaskHandle = nullptr;
extern "C" void microRosTask(void* arg);

inline etl::array<TaskHandle_t, 4> motorTaskHandles{};
extern "C" void motorTask(void* arg);

} // namespace task
} // namespace freertos

#endif // TASKS_HPP