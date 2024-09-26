/**
 * @file subscriber.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Source file of MicroRos subscriber related functions and callbacks.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "subscriber.hpp"
#include <rclc/subscription.h>

#include <FreeRTOS.h>
#include <queue.h>

namespace ros {

void gripperSubscriberCallback(const void* msgin, void* context) {
    if (context == nullptr || msgin == nullptr) {
        return;
    }
    auto driveQueue = static_cast<QueueHandle_t>(context);
    auto msg = static_cast<const rover_drive_interfaces__msg__MotorDrive*>(msgin);
    xQueueOverwrite(driveQueue, msg);
}

void armStepperSubscriberCallback(const void* msgin, void* context) {
    if (context == nullptr || msgin == nullptr) {
        return;
    }
    auto armStepperQueue = static_cast<QueueHandle_t>(context);
    auto msg = static_cast<const rover_arm_interfaces__msg__ArmStepper*>(msgin);
    xQueueOverwrite(armStepperQueue, msg);
}

Subscriber::Subscriber(
    rcl_node_t* node, etl::string_view name, const rosidl_message_type_support_t* type)
    : subscriber_(rcl_get_zero_initialized_subscription()) {
    rclc_subscription_init_best_effort(&subscriber_, node, type, name.data());
}

rcl_ret_t Subscriber::addToExecutor(rclc_executor_t* executor, void* msg,
    rclc_subscription_callback_with_context_t callback, void* context,
    rclc_executor_handle_invocation_t event) {
    return rclc_executor_add_subscription_with_context(
        executor, &subscriber_, msg, callback, context, event);
}

etl::array<Subscriber, 3> createStepperSubscribers(rcl_node_t* node) {
    // Create the MicroROS arm stepper subscribers.
    const auto armStepperMsgType =
        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_arm_interfaces, msg, ArmStepper);
    constexpr etl::array armStepperSubscriberNames{ "arm_stepper_subscriber_0",
        "arm_stepper_subscriber_1", "arm_stepper_subscriber_2" };

    return { ros::Subscriber(node, armStepperSubscriberNames[0], armStepperMsgType),
        ros::Subscriber(node, armStepperSubscriberNames[1], armStepperMsgType),
        ros::Subscriber(node, armStepperSubscriberNames[2], armStepperMsgType) };
}

etl::array<Subscriber, 3> createGripperSubscribers(rcl_node_t* node) {
    // Create the MicroROS arm stepper subscribers.
    const auto gripperMsgType =
        ROSIDL_GET_MSG_TYPE_SUPPORT(rover_drive_interfaces, msg, MotorDrive);
    constexpr etl::array armGripperSubscriberNames{ "arm_gripper_subscriber_0",
        "arm_gripper_subscriber_1", "arm_gripper_subscriber_2" };

    return { ros::Subscriber(node, armGripperSubscriberNames[0], gripperMsgType),
        ros::Subscriber(node, armGripperSubscriberNames[1], gripperMsgType),
        ros::Subscriber(node, armGripperSubscriberNames[2], gripperMsgType) };
}


} // namespace ros