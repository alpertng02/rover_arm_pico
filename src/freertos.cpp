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
#include <rover_arm_interfaces/msg/arm_stepper.h>
#include <rover_arm_interfaces/msg/stepper_feedback.h>

#include <etl/string_view.h>

namespace freertos {

void createMsgQueues() {
    // Create all the queues.
    for (int i = 0; i < queue::gripperFeedbackQueues.size(); i++) {
        queue::gripperFeedbackQueues[i] =
            xQueueCreate(1, sizeof(rover_drive_interfaces__msg__MotorFeedback));
        queue::gripperMotorQueues[i] =
            xQueueCreate(1, sizeof(rover_drive_interfaces__msg__MotorDrive));
        queue::armStepperQueues[i] = xQueueCreate(1, sizeof(rover_arm_interfaces__msg__ArmStepper));
        queue::stepperFeedbackQueues[i] =
            xQueueCreate(1, sizeof(rover_arm_interfaces__msg__StepperFeedback));
    }
}

} // namespace freertos
