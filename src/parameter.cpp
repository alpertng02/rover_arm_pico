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
#include <etl/unordered_map.h>

namespace ros {

namespace parameter {

constexpr static etl::string_view maxMotorRpmName{ "max_motor_rpm" };
constexpr static rclc_parameter_type_t maxMotorRpmType = RCLC_PARAMETER_INT;

constexpr static etl::string_view maxMotorDutyCycleName{ "max_motor_dutycycle" };
constexpr static rclc_parameter_type_t maxMotorDutyCycleType = RCLC_PARAMETER_DOUBLE;

constexpr static etl::string_view maxMotorCurrentName{ "max_motor_current" };
constexpr static rclc_parameter_type_t maxMotorCurrentType = RCLC_PARAMETER_DOUBLE;

constexpr static etl::string_view motorTimeoutMsName{ "motor_timeout_ms" };
constexpr static rclc_parameter_type_t motorTimeoutMsType = RCLC_PARAMETER_INT;

constexpr static etl::string_view stepperTimeoutMsName{ "stepper_timeout_ms" };
constexpr static rclc_parameter_type_t stepperTimeoutMsType = RCLC_PARAMETER_INT;

constexpr static etl::string_view feedbackPeriodMsName{ "feedback_period_ms" };
constexpr static rclc_parameter_type_t feedbackPeriodMsType = RCLC_PARAMETER_INT;

constexpr static etl::array<etl::string_view, 3> stepperGearRatiosNames{ "stepper_gear_ratio_0",
    "stepper_gear_ratio_1", "stepper_gear_ratio_2" };
constexpr static rclc_parameter_type_t stepperGearRatiosType = RCLC_PARAMETER_INT;

constexpr static etl::array<etl::string_view, 3> stepperStepsPerRevNames{ "stepper_steps_per_rev_0",
    "stepper_steps_per_rev_1", "stepper_steps_per_rev_2" };
constexpr static rclc_parameter_type_t stepperStepsPerRevType = RCLC_PARAMETER_INT;

constexpr static etl::array<etl::string_view, 3> stepperSpeedControlPeriodMsNames{
    "stepper_speed_control_period_ms_0", "stepper_speed_control_period_ms_1",
    "stepper_speed_control_period_ms_2"
};
constexpr static rclc_parameter_type_t stepperSpeedControlPeriodMsType = RCLC_PARAMETER_INT;

constexpr static etl::array<etl::string_view, 3> stepperMaxAccelNames{ "stepper_max_accel_0",
    "stepper_max_accel_1", "stepper_max_accels_2" };
constexpr static rclc_parameter_type_t stepperMaxAccelType = RCLC_PARAMETER_INT;


static etl::unordered_map<etl::string_view, void*, 18> parameterMap{
    {maxMotorRpmName,                      &maxMotorRpm                   },
    { maxMotorDutyCycleName,               &maxMotorDutyCycle             },
    { maxMotorCurrentName,                 &maxMotorCurrent               },
    { motorTimeoutMsName,                  &motorTimeoutMs                },
    { stepperTimeoutMsName,                &stepperTimeoutMs              },
    { feedbackPeriodMsName,                &feedbackPeriodMs              },
    { stepperGearRatiosNames[0],           &stepperGearRatios[0]          },
    { stepperGearRatiosNames[1],           &stepperGearRatios[1]          },
    { stepperGearRatiosNames[2],           &stepperGearRatios[2]          },
    { stepperStepsPerRevNames[0],          &stepperStepsPerRev[0]         },
    { stepperStepsPerRevNames[1],          &stepperStepsPerRev[1]         },
    { stepperStepsPerRevNames[2],          &stepperStepsPerRev[2]         },
    { stepperSpeedControlPeriodMsNames[0], &stepperSpeedControlPeriodMs[0]},
    { stepperSpeedControlPeriodMsNames[1], &stepperSpeedControlPeriodMs[1]},
    { stepperSpeedControlPeriodMsNames[2], &stepperSpeedControlPeriodMs[2]},
    { stepperMaxAccelNames[0],             &stepperMaxAccel[0]            },
    { stepperMaxAccelNames[1],             &stepperMaxAccel[1]            },
    { stepperMaxAccelNames[2],             &stepperMaxAccel[2]            },
};

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
    ret += rclc_add_parameter(&paramServer_, motorTimeoutMsName.data(), motorTimeoutMsType);
    ret += rclc_add_parameter(&paramServer_, stepperTimeoutMsName.data(), stepperTimeoutMsType);
    ret += rclc_add_parameter(&paramServer_, feedbackPeriodMsName.data(), feedbackPeriodMsType);

    for (int i = 0; i < stepperGearRatios.size(); i++) {
        ret += rclc_add_parameter(
            &paramServer_, stepperGearRatiosNames[i].data(), stepperGearRatiosType);
        ret += rclc_add_parameter(
            &paramServer_, stepperStepsPerRevNames[i].data(), stepperStepsPerRevType);
        ret += rclc_add_parameter(&paramServer_, stepperSpeedControlPeriodMsNames[i].data(),
            stepperSpeedControlPeriodMsType);
        ret +=
            rclc_add_parameter(&paramServer_, stepperMaxAccelNames[i].data(), stepperMaxAccelType);
    }

    // Set the values of the parameters in &paramServer_.
    ret += rclc_parameter_set_int(&paramServer_, maxMotorRpmName.data(), maxMotorRpm);
    ret +=
        rclc_parameter_set_double(&paramServer_, maxMotorDutyCycleName.data(), maxMotorDutyCycle);
    ret += rclc_parameter_set_double(&paramServer_, maxMotorCurrentName.data(), maxMotorCurrent);
    ret += rclc_parameter_set_int(&paramServer_, motorTimeoutMsName.data(), motorTimeoutMs);
    ret += rclc_parameter_set_int(&paramServer_, stepperTimeoutMsName.data(), stepperTimeoutMs);
    ret += rclc_parameter_set_int(&paramServer_, feedbackPeriodMsName.data(), feedbackPeriodMs);

    for (int i = 0; i < stepperGearRatios.size(); i++) {
        ret += rclc_parameter_set_int(
            &paramServer_, stepperGearRatiosNames[i].data(), stepperGearRatios[i]);
        ret += rclc_parameter_set_int(
            &paramServer_, stepperStepsPerRevNames[i].data(), stepperStepsPerRev[i]);
        ret += rclc_parameter_set_int(&paramServer_, stepperSpeedControlPeriodMsNames[i].data(),
            stepperSpeedControlPeriodMs[i]);
        ret += rclc_parameter_set_int(
            &paramServer_, stepperMaxAccelNames[i].data(), stepperMaxAccel[i]);
    }
    return ret;
}

bool onParameterChange(const Parameter* oldParam, const Parameter* newParam, void* context) {
    auto paramServer = static_cast<parameter::Server*>(context);
    if (oldParam == nullptr || newParam == nullptr || paramServer == nullptr) {
        return false;
    }

    void* res = parameterMap[newParam->name.data];
    if (res != nullptr) {
        switch (newParam->value.type) {
            case RCLC_PARAMETER_INT:
                *static_cast<int32_t*>(res) = newParam->value.integer_value;
                break;
            case RCLC_PARAMETER_DOUBLE:
                *static_cast<float*>(res) = newParam->value.double_value;
                break;
            case RCLC_PARAMETER_BOOL:
                *static_cast<bool*>(res) = newParam->value.bool_value;
                break;
        }
        return true;
    }
    return false;
}

} // namespace parameter
} // namespace ros