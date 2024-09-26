#include "tasks.hpp"
#include "parameters.hpp"
#include "publisher.hpp"
#include "queues.hpp"
#include "rmw_microros/ping.h"
#include "subscriber.hpp"

#include <rover_drive_interfaces/msg/motor_feedback.h>
#include <rover_drive_interfaces/msg/motor_drive.h>
#include <rover_arm_interfaces/msg/arm_stepper.h>
#include <rover_arm_interfaces/msg/stepper_feedback.h>

#include "BTS7960.hpp"
#include "stepper.hpp"
#include "pinout.hpp"

#include <hardware/watchdog.h>

namespace freertos {

namespace task {

template <uint i> void gripperMotorTask(void* arg) {
    // Create the motor and encoder classes.
    motor::BTS7960 motor(pinout::gripperMotorPwmL[i], pinout::gripperMotorPwmR[i]);

    // Create the ros messeages.
    rover_drive_interfaces__msg__MotorDrive gripperMsgReceived{};
    rover_drive_interfaces__msg__MotorFeedback feedbackMsgSent{};

    // Integral and previous error to use for PID.
    float integral = 0.0f;
    float errorRpmPrev = 0.0f;

    absolute_time_t lastMsgReceivedTime = get_absolute_time();
    // Get the starting time of the task.
    TickType_t startTick{ xTaskGetTickCount() };
    while (true) {
        if (xQueueReceive(freertos::queue::gripperMotorQueues[i],
                &gripperMsgReceived,
                ros::parameter::motorTimeoutMs) != pdTRUE) {
            feedbackMsgSent.dutycycle = 0;
        } else {
            // Assume there is a linear relationship between the
            // motor dutycyle and RPM and use open loop control.
            feedbackMsgSent.dutycycle = gripperMsgReceived.target_rpm *
                                        ros::parameter::maxMotorDutyCycleUpperConstraint /
                                        ros::parameter::maxMotorRpm;
        }
        // Clamp the dutycycle within the boundaries.
        feedbackMsgSent.dutycycle = etl::clamp(feedbackMsgSent.dutycycle,
            -ros::parameter::maxMotorDutyCycle,
            ros::parameter::maxMotorDutyCycle);

        // Set the dutycyle of the motors.
        motor.setSpeed(feedbackMsgSent.dutycycle);
        // Send the feedback messeage to the queue
        xQueueOverwrite(freertos::queue::gripperFeedbackQueues[i], &feedbackMsgSent);
    }
}
template <uint i> void stepperMotorTask(void* arg) {

    uint32_t speedControlPeriodMs = ros::parameter::stepperSpeedControlPeriodMs[i];
    uint32_t stepsPerRev = ros::parameter::stepperStepsPerRev[i];

    Stepper stepper(
        pinout::armStepperPul[i], pinout::armStepperDir[i], stepsPerRev, speedControlPeriodMs);
    
    rover_arm_interfaces__msg__ArmStepper armStepperMsgReceived{};
    rover_arm_interfaces__msg__StepperFeedback stepperFeedbackSent{};

    while (true) {
        if (xQueueReceive(queue::armStepperQueues[i],
                &armStepperMsgReceived,
                ros::parameter::stepperTimeoutMs) != pdTRUE) {
            stepper.enable(false);
        } else {
            // stepper.startMotion(armStepperMsgReceived.target_pos_steps,
            //     ros::parameter::stepperMaxAccel[i], 1000, true);
            stepper.setSpeed(armStepperMsgReceived.speed_steps_sec);
        }

        if (speedControlPeriodMs != ros::parameter::stepperSpeedControlPeriodMs[i]) {
            speedControlPeriodMs = ros::parameter::stepperSpeedControlPeriodMs[i];
            stepper.setTimerPeriod(speedControlPeriodMs);
        }
        if (stepsPerRev != ros::parameter::stepperStepsPerRev[i]) {
            stepsPerRev = ros::parameter::stepperStepsPerRev[i];
            stepper.setStepsPerRev(stepsPerRev);
        }
        stepperFeedbackSent.pos_steps = stepper.getPos();
        // stepperFeedbackSent.pos_steps = armStepperMsgReceived.target_pos_steps;
        stepperFeedbackSent.speed_steps_sec = stepper.getActualSpeed();

        xQueueOverwrite(queue::stepperFeedbackQueues[i], &stepperFeedbackSent);
    }
}

#define RCLCHECK(x)                                                                                \
    do {                                                                                           \
        if (x == RCL_RET_OK) {                                                                     \
            watchdog_update();                                                                     \
        } else {                                                                                   \
            while (true) {                                                                         \
                tight_loop_contents();                                                             \
            }                                                                                      \
        }                                                                                          \
    } while (0)

void microRosTask(void* arg) {
    // MicroRos boiler plate.
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support{};
    while (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
        rmw_uros_ping_agent(200, 1);
    }

    gpio_put(pinout::led, true);

    rcl_ret_t ret = 0;

    rcl_node_t node = rcl_get_zero_initialized_node();
    ret += rclc_node_init_default(&node, "pico_node", "arm", &support);

    // Create the MicroROS motor feedback publishers
    ret += ros::createMotorFeedbackPublishers(&node);
    ret += ros::createStepperFeedbackPublishers(&node);

    auto armStepperSubscribers{ ros::createStepperSubscribers(&node) };
    auto gripperSubscribers{ ros::createGripperSubscribers(&node) };

    // Create MicroROS timer that will publish the feedback messeages
    // We use a MicroROS task for publishing instead of publishing directly in
    // FreeRTOS tasks as I had problem with random crashes when publishing from
    // different cores at the same time.
    rcl_timer_t publisherTimer = rcl_get_zero_initialized_timer();
    ret += rclc_timer_init_default(&publisherTimer, &support,
        RCL_MS_TO_NS(ros::parameter::feedbackPeriodMs), ros::publisherTimerCallback);

    // Create the executor responsible for all the subscribers and the timer.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    ret += rclc_executor_init(&executor, &support.context,
        armStepperSubscribers.size() + gripperSubscribers.size() + 1, &allocator);

    etl::array<rover_drive_interfaces__msg__MotorDrive, 3> gripperMsgs{};
    etl::array<rover_arm_interfaces__msg__ArmStepper, 3> armStepperMsgs{};
    // Add the subscribers and the timer to the executor.
    for (int i = 0; i < armStepperSubscribers.size(); i++) {
        gripperSubscribers[i].addToExecutor(&executor, &gripperMsgs[i],
            ros::gripperSubscriberCallback, queue::gripperMotorQueues[i], ON_NEW_DATA);
        armStepperSubscribers[i].addToExecutor(&executor, &armStepperMsgs[i],
            ros::armStepperSubscriberCallback, queue::armStepperQueues[i], ON_NEW_DATA);
    }
    ret += rclc_executor_add_timer(&executor, &publisherTimer);

    // Create the parameter server and a seperate executor for the parameter
    // server.
    // We need the second executor since we have more handles than the maximum
    // executor handle.
    ros::parameter::Server paramServer(&node, true, 18, true, false);
    rclc_executor_t paramServerExecutor = rclc_executor_get_zero_initialized_executor();
    ret += rclc_executor_init(
        &paramServerExecutor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);
    ret += paramServer.addToExecutor(&paramServerExecutor);

    constexpr uint32_t watchdogTimeoutMs = 1000;
    watchdog_enable(watchdogTimeoutMs, true);

    RCLCHECK(ret);
    while (true) {
        // Spin the executors to check if there are new subscriber messeages or
        // parameter server requests.
        const auto gripperExecutorResult = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        const auto paramExecutorResult =
            rclc_executor_spin_some(&paramServerExecutor, RCL_MS_TO_NS(1));
        // Update the watchdog
        watchdog_update();

        // Delay the tasks to free the core for other tasks.
        // TODO add parameter to control executor period.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
} // namespace task

void createMicroRosTask() {

    constexpr uint32_t microRosTaskStack = 4000;
    constexpr uint32_t microRosTaskPriority = configMAX_PRIORITIES - 2;
    constexpr uint32_t microRosTaskCoreAffinity = 0x01;

    constexpr etl::string_view microRosTaskName{ "micro_ros_task" };
    xTaskCreateAffinitySet(task::microRosTask, microRosTaskName.data(), microRosTaskStack, nullptr,
        microRosTaskPriority, microRosTaskCoreAffinity, &task::microRosTaskHandle);
}


void createGripperMotorTasks() {

    constexpr uint32_t gripperMotorTaskStack = 256;
    constexpr uint32_t gripperMotorTaskPriority = configMAX_PRIORITIES - 4;
    constexpr uint32_t gripperMotorTaskCoreAffinity = 0x03;
    constexpr std::array gripperMotorTasks{ task::gripperMotorTask<0>, task::gripperMotorTask<1>,
        task::gripperMotorTask<2> };

    constexpr etl::array taskNames{ "gripper_motor_task_0", "gripper_motor_task_1",
        "gripper_motor_task_2" };
    // This indexes represent which publisher and subscriber queue the motor task
    // is going to use.
    // Even though these values are pointers, we only use the pointers as
    // numerical values instead of adresses because FreeRTOS tasks take a void*
    // argument for custom data.
    for (int i = 0; i < taskNames.size(); i++) {
        xTaskCreateAffinitySet(gripperMotorTasks[i], taskNames[i], gripperMotorTaskStack, nullptr,
            gripperMotorTaskPriority, gripperMotorTaskCoreAffinity,
            &task::gripperMotorTaskHandles[i]);
    }
}

void createStepperMotorTasks() {

    constexpr uint32_t stepperTaskStack = 512;
    constexpr uint32_t stepperTaskPriority = configMAX_PRIORITIES - 3;
    constexpr std::array<uint32_t, 3> stepperTaskCoreAffinity = { 0x03, 0x03, 0x03 };
    constexpr std::array stepperMotorTasks{ task::stepperMotorTask<0>, task::stepperMotorTask<1>,
        task::stepperMotorTask<2> };

    constexpr etl::array taskNames{ "stepper_motor_task_0", "stepper_motor_task_1",
        "stepper_motor_task_2" };
    // This indexes represent which publisher and subscriber queue the motor task
    // is going to use.
    // Even though these values are pointers, we only use the pointers as
    // numerical values instead of adresses because FreeRTOS tasks take a void*
    // argument for custom data.
    for (int i = 0; i < taskNames.size(); i++) {
        xTaskCreateAffinitySet(stepperMotorTasks[i], taskNames[i], stepperTaskStack, nullptr,
            stepperTaskPriority, stepperTaskCoreAffinity[i], &task::stepperMotorTaskHandles[i]);
    }
}


} // namespace freertos