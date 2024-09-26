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

#include <etl/string_view.h>
namespace ros {

class Publisher {
public:
    Publisher() = default;

    Publisher(
        rcl_node_t* node, etl::string_view name, const rosidl_message_type_support_t* typeSupport);

    rcl_ret_t init(
        rcl_node_t* node, etl::string_view name, const rosidl_message_type_support_t* typeSupport);

    rcl_ret_t publish(const void* msg, rmw_publisher_allocation_t* allocation = nullptr);

private:
    rcl_publisher_t publisher_{ rcl_get_zero_initialized_publisher() };
};


extern "C" void publisherTimerCallback(rcl_timer_t* timer, int64_t last_call_time);

rcl_ret_t createMotorFeedbackPublishers(rcl_node_t* node);
rcl_ret_t createStepperFeedbackPublishers(rcl_node_t* node);
} // namespace ros

#endif // PUBLISHER_HPP