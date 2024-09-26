/**
 * @file publisher.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Source file of MicroROS publisher related functions and callbacks.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "publisher.hpp"

#include <rclc/publisher.h>
#include <rover_drive_interfaces/msg/motor_feedback.h>
#include <rover_arm_interfaces/msg/stepper_feedback.h>

#include <etl/array.h>

#include "parameters.hpp"
#include "queues.hpp"

namespace ros {

Publisher::Publisher(
    rcl_node_t* node, etl::string_view name, const rosidl_message_type_support_t* typeSupport) {
    if (node != nullptr) {
        rclc_publisher_init_default(&publisher_, node, typeSupport, name.data());
    }
}

rcl_ret_t Publisher::init(
    rcl_node_t* node, etl::string_view name, const rosidl_message_type_support_t* typeSupport) {
    return rclc_publisher_init_best_effort(&publisher_, node, typeSupport, name.data());
}


rcl_ret_t Publisher::publish(const void* msg, rmw_publisher_allocation_t* allocation) {
    return rcl_publish(&publisher_, msg, allocation);
}


static etl::array<Publisher, 3> motorFeedbackPublishers{};
rcl_ret_t createMotorFeedbackPublishers(rcl_node_t* node) {
    constexpr etl::array gripperPublisherNames{
        "gripper_motor_feedback_0",
        "gripper_motor_feedback_1",
        "gripper_motor_feedback_2",
    };
    const auto motorFeedbackMsgType =
        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_drive_interfaces, msg, MotorFeedback);
    rcl_ret_t ret = 0;
    for (int i = 0; i < gripperPublisherNames.size(); i++) {
        ret +=
            motorFeedbackPublishers[i].init(node, gripperPublisherNames[i], motorFeedbackMsgType);
    }
    return ret;
}

static etl::array<Publisher, 3> stepperFeedbackPublishers{};
rcl_ret_t createStepperFeedbackPublishers(rcl_node_t* node) {
    constexpr etl::array stepperPublisherNames{
        "stepper_motor_feedback_0",
        "stepper_motor_feedback_1",
        "stepper_motor_feedback_2",
    };
    const auto stepperFeedbackMsgType =
        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_arm_interfaces, msg, StepperFeedback);
    rcl_ret_t ret = 0;
    for (int i = 0; i < stepperPublisherNames.size(); i++) {
        ret += stepperFeedbackPublishers[i].init(
            node, stepperPublisherNames[i], stepperFeedbackMsgType);
    }
    return ret;
}

void publisherTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
    // If timer period was changed through parameters, set the new period to the
    // timer.
    int64_t lastPeriod{};
    rcl_ret_t ret = rcl_timer_get_period(timer, &lastPeriod);
    if (lastPeriod != RCL_MS_TO_NS(parameter::feedbackPeriodMs)) {
        ret += rcl_timer_exchange_period(
            timer, RCL_MS_TO_NS(parameter::feedbackPeriodMs), &lastPeriod);
    }
    // Receive all the feedback messeages from the queues and publish them.
    rover_drive_interfaces__msg__MotorFeedback motorFeedbackMsgBuffer{};
    rover_arm_interfaces__msg__StepperFeedback stepperFeedbackMsgBuffer{};
    for (int i = 0; i < motorFeedbackPublishers.size(); i++) {
        if (xQueuePeek(freertos::queue::stepperFeedbackQueues[i], &stepperFeedbackMsgBuffer, 0) ==
            pdTRUE) {
            ret += stepperFeedbackPublishers[i].publish(&stepperFeedbackMsgBuffer);
        }
        if (xQueuePeek(freertos::queue::gripperFeedbackQueues[i], &motorFeedbackMsgBuffer, 0) ==
            pdTRUE) {
            ret += motorFeedbackPublishers[i].publish(&motorFeedbackMsgBuffer);
        }
    }
}
} // namespace ros