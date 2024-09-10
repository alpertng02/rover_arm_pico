/**
 * @file freertos.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Source file of Freertos related functions and tasks.
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "freertos.hpp"
#include "queues.hpp"
#include "tasks.hpp"

#include <rover_drive_interfaces/msg/motor_drive.h>
#include <rover_drive_interfaces/msg/motor_feedback.h>

#include <etl/string_view.h>

namespace freertos {

void createMsgQueues() {
    // Create all the queues.
    for (int i = 0; i < queue::publisherQueues.size(); i++) {
        queue::publisherQueues[i] =
            xQueueCreate(1, sizeof(rover_drive_interfaces__msg__MotorFeedback));
        queue::driveQueues[i] = xQueueCreate(1, sizeof(rover_drive_interfaces__msg__MotorDrive));
    }
}
void createMicroRosTask() {

    constexpr uint32_t microRosTaskStack = 4000;
    constexpr uint32_t microRosTaskPriority = configMAX_PRIORITIES - 2;
    constexpr uint32_t microRosTaskCoreAffinity = 0x01;

    constexpr etl::string_view microRosTaskName{ "micro_ros_task" };
    xTaskCreateAffinitySet(task::microRosTask, microRosTaskName.data(), microRosTaskStack, nullptr,
        microRosTaskPriority, microRosTaskCoreAffinity, &task::microRosTaskHandle);
}

void createMotorTasks() {

    constexpr uint32_t motorTaskStack = 500;
    constexpr uint32_t motorTaskPriority = configMAX_PRIORITIES - 3;
    constexpr uint32_t motorTaskCoreAffinity = 0x03;

    constexpr etl::array<etl::string_view, 4> taskNames{ "motor_task_0", "motor_task_1",
        "motor_task_2", "motor_task_3" };
    // This indexes represent which publisher and subscriber queue the motor task
    // is going to use.
    // Even though these values are pointers, we only use the pointers as
    // numerical values instead of adresses because FreeRTOS tasks take a void*
    // argument for custom data.
    void* indexes[4]{ (void*)0, (void*)1, (void*)2, (void*)3 };
    for (int i = 0; i < taskNames.size(); i++) {
        xTaskCreateAffinitySet(task::motorTask, taskNames[i].data(), motorTaskStack, indexes[i],
            motorTaskPriority, motorTaskCoreAffinity, &task::motorTaskHandles[i]);
    }
}
} // namespace freertos
