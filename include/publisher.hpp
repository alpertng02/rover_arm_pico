/**
 * @file publisher.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Header file for MicroROS publisher related functions and callbacks
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

#include <rcl/publisher.h>
#include <rcl/timer.h>
#include <rover_drive_interfaces/msg/motor_feedback.h>

#include <etl/array.h>

namespace ros {

inline etl::array<rcl_publisher_t, 4> motorFeedbackPublishers{};
inline etl::array<rover_drive_interfaces__msg__MotorFeedback, 4> motorFeedbackMsgs{};

extern "C" void publisherTimerCallback(rcl_timer_t* timer, int64_t last_call_time);

rcl_ret_t createMotorFeedbackPublishers(rcl_node_t* node);

} // namespace ros

#endif // PUBLISHER_HPP