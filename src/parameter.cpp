/**
 * @file parameter.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Source file of MicroROS parameter related functions.
 * @version 0.1
 * @date 2024-09-09
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "parameters.hpp"
#include <etl/string_view.h>

namespace ros {

namespace parameter {

constexpr static etl::string_view maxMotorRpmName{ "max_motor_rpm" };
constexpr static rclc_parameter_type_t maxMotorRpmType = RCLC_PARAMETER_INT;

constexpr static etl::string_view maxMotorDutyCycleName{ "max_motor_dutycycle" };
constexpr static rclc_parameter_type_t maxMotorDutyCycleType = RCLC_PARAMETER_DOUBLE;

constexpr static etl::string_view maxMotorCurrentName{ "max_motor_current" };
constexpr static rclc_parameter_type_t maxMotorCurrentType = RCLC_PARAMETER_DOUBLE;

constexpr static etl::string_view motorPidLoopPeriodMsName{ "motor_pid_loop_period_ms" };
constexpr static rclc_parameter_type_t motorPidLoopPeriodMsType = RCLC_PARAMETER_INT;

constexpr static etl::string_view motorTimeoutMsName{ "motor_timeout_ms" };
constexpr static rclc_parameter_type_t motorTimeoutMsType = RCLC_PARAMETER_INT;

constexpr static etl::string_view motorFeedbackPeriodMsName{ "motor_feedback_period_ms" };
constexpr static rclc_parameter_type_t motorFeedbackPeriodMsType = RCLC_PARAMETER_INT;

constexpr static etl::string_view motorPidModeName{ "motor_pid_mode" };
constexpr static rclc_parameter_type_t motorPidModeType = RCLC_PARAMETER_BOOL;

constexpr static etl::string_view motorPidKpName{ "motor_pid_kp" };
constexpr static rclc_parameter_type_t motorPidKpType = RCLC_PARAMETER_DOUBLE;

constexpr static etl::string_view motorPidKiName{ "motor_pid_ki" };
constexpr static rclc_parameter_type_t motorPidKiType = RCLC_PARAMETER_DOUBLE;

constexpr static etl::string_view motorPidKdName{ "motor_pid_kd" };
constexpr static rclc_parameter_type_t motorPidKdType = RCLC_PARAMETER_DOUBLE;

extern "C" bool onParameterChange(
    const Parameter* oldParam, const Parameter* newParam, void* context);

Server::Server(rcl_node_t* node, rclc_parameter_options_t* serverOptions) {
    if (serverOptions == nullptr) {
        rclc_parameter_server_init_default(&paramServer_, node);
    } else {
        rclc_parameter_server_init_with_option(&paramServer_, node, serverOptions);
    }
}

Server::Server(rcl_node_t* node, bool notify_changed_over_dds, uint32_t max_params,
    bool allow_undeclared_parameters, bool low_mem_mode) {
    rclc_parameter_options_t options{ .notify_changed_over_dds = notify_changed_over_dds,
        .max_params = max_params,
        .allow_undeclared_parameters = allow_undeclared_parameters,
        .low_mem_mode = low_mem_mode };
    rclc_parameter_server_init_with_option(&paramServer_, node, &options);
}

rcl_ret_t Server::addToExecutor(rclc_executor_t* executor) {
    return rclc_executor_add_parameter_server_with_context(
        executor, &paramServer_, onParameterChange, this);
}

rcl_ret_t Server::initParameters() {
    rcl_ret_t ret = 0;
    // Add parameters to the server.
    ret += rclc_add_parameter(&paramServer_, maxMotorRpmName.data(), maxMotorRpmType);
    ret += rclc_add_parameter(&paramServer_, maxMotorDutyCycleName.data(), maxMotorDutyCycleType);
    ret += rclc_add_parameter_constraint_double(&paramServer_, maxMotorDutyCycleName.data(),
        maxMotorDutyCycleLowerConstraint, maxMotorDutyCycleUpperConstraint, 0);

    ret += rclc_add_parameter(&paramServer_, maxMotorCurrentName.data(), maxMotorCurrentType);
    ret += rclc_add_parameter(
        &paramServer_, motorPidLoopPeriodMsName.data(), motorPidLoopPeriodMsType);
    ret += rclc_add_parameter(&paramServer_, motorTimeoutMsName.data(), motorTimeoutMsType);
    ret += rclc_add_parameter(
        &paramServer_, motorFeedbackPeriodMsName.data(), motorFeedbackPeriodMsType);
    ret += rclc_add_parameter(&paramServer_, motorPidModeName.data(), motorPidModeType);

    ret += rclc_add_parameter(&paramServer_, motorPidKpName.data(), motorPidKpType);
    ret += rclc_add_parameter(&paramServer_, motorPidKdName.data(), motorPidKiType);
    ret += rclc_add_parameter(&paramServer_, motorPidKiName.data(), motorPidKdType);

    // Set the values of the parameters in &paramServer_.
    ret += rclc_parameter_set_int(&paramServer_, maxMotorRpmName.data(), maxMotorRpm);
    ret +=
        rclc_parameter_set_double(&paramServer_, maxMotorDutyCycleName.data(), maxMotorDutyCycle);
    ret += rclc_parameter_set_double(&paramServer_, maxMotorCurrentName.data(), maxMotorCurrent);
    ret += rclc_parameter_set_int(
        &paramServer_, motorPidLoopPeriodMsName.data(), motorPidLoopPeriodMs);
    ret += rclc_parameter_set_int(&paramServer_, motorTimeoutMsName.data(), motorTimeoutMs);
    ret += rclc_parameter_set_int(
        &paramServer_, motorFeedbackPeriodMsName.data(), motorFeedbackPeriodMs);
    ret += rclc_parameter_set_bool(&paramServer_, motorPidModeName.data(), motorPidMode);
    ret += rclc_parameter_set_double(&paramServer_, motorPidKpName.data(), motorPidKp);
    ret += rclc_parameter_set_double(&paramServer_, motorPidKiName.data(), motorPidKi);
    ret += rclc_parameter_set_double(&paramServer_, motorPidKdName.data(), motorPidKd);
    return ret;
}

bool onParameterChange(const Parameter* oldParam, const Parameter* newParam, void* context) {
    auto paramServer = static_cast<parameter::Server*>(context);
    if (oldParam == nullptr || newParam == nullptr || paramServer == nullptr) {
        return false;
    }

    // TODO convert this to a hashmap
    switch (newParam->value.type) {
        case RCLC_PARAMETER_INT:
            if (maxMotorRpmName == newParam->name.data) {
                maxMotorRpm = newParam->value.integer_value;
            } else if (motorPidLoopPeriodMsName == newParam->name.data) {
                motorPidLoopPeriodMs = newParam->value.integer_value;
            } else if (motorFeedbackPeriodMsName == newParam->name.data) {
                motorFeedbackPeriodMs = newParam->value.integer_value;
            } else if (motorTimeoutMsName == newParam->name.data) {
                motorTimeoutMs = newParam->value.integer_value;
            } else {
                return false;
            }
            break;
        case RCLC_PARAMETER_DOUBLE:
            if (maxMotorDutyCycleName == newParam->name.data) {
                maxMotorDutyCycle = newParam->value.double_value;
            } else if (maxMotorCurrentName == newParam->name.data) {
                maxMotorCurrent = newParam->value.double_value;
            } else if (motorPidKpName == newParam->name.data) {
                motorPidKp = newParam->value.double_value;
            } else if (motorPidKiName == newParam->name.data) {
                motorPidKi = newParam->value.double_value;
            } else if (motorPidKdName == newParam->name.data) {
                motorPidKd = newParam->value.double_value;
            } else {
                return false;
            }
            break;
        case RCLC_PARAMETER_BOOL:
            if (motorPidModeName == newParam->name.data) {
                motorPidMode = newParam->value.bool_value;
            } else {
                return false;
            }
            break;
        default:
            return false;
    }

    return true;
}

} // namespace parameter
} // namespace ros