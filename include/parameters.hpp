/**
 * @file parameters.hpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Header file of MicroROS parameters and server.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <pico/types.h>
#include <rclc_parameter/rclc_parameter.h>
#include <etl/array.h>

namespace ros {

namespace parameter {

class Server {
public:
    Server(rcl_node_t* node, rclc_parameter_options_t* serverOptions = nullptr);

    Server(rcl_node_t* node, bool notify_changed_over_dds, uint32_t max_params,
        bool allow_undeclared_parameters, bool low_mem_mode);

    rcl_ret_t addToExecutor(rclc_executor_t* executor);
    rcl_ret_t initParameters();

private:
    rclc_parameter_server_t paramServer_{};
};

inline int32_t maxMotorRpm = 100;

inline float maxMotorDutyCycle = 100.0f;
constexpr inline float maxMotorDutyCycleUpperConstraint = 100.0f;
constexpr inline float maxMotorDutyCycleLowerConstraint = 0.0f;

inline float maxMotorCurrent = 50.0f;

inline int32_t motorTimeoutMs = 1000;
inline int32_t stepperTimeoutMs = 1000;

inline int32_t feedbackPeriodMs = 100;

inline etl::array<float, 3> stepperGearRatios{ 50.0f, 68.18181818f, 2.0f };
inline etl::array<int32_t, 3> stepperStepsPerRev{ 400, 400, 2000 };
inline etl::array<int32_t, 3> stepperSpeedControlPeriodMs{ 2, 2, 2 };

inline etl::array<int32_t, 3> stepperMaxAccel {
    stepperStepsPerRev[0] * stepperStepsPerRev[0] / 2,
    stepperStepsPerRev[1] * stepperStepsPerRev[1] / 2,
    stepperStepsPerRev[2] * stepperStepsPerRev[2] / 2
};


} // namespace parameter

} // namespace ros

#endif // PARAMETERS_HPP