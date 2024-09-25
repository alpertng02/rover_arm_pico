#include "tasks.hpp"
#include "freertos.hpp"
#include "parameters.hpp"
#include "publisher.hpp"
#include "queues.hpp"
#include "subscriber.hpp"

#include "BTS7960.hpp"
#include "stepper.hpp"
#include "pinout.hpp"

#include <hardware/watchdog.h>

namespace freertos {

namespace task {

void motorTask(void* arg) {
    // Convert the index into size_t. This void* arg is the value we got from
    // indexes[i] in createMotorTasks()
    const size_t i = (size_t)(arg);

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

void stepperTask(void* arg) {
    if (arg == nullptr) {
        vTaskDelete(nullptr);
    }
    const size_t i = (size_t)arg;

    uint32_t speedControlPeriodMs = ros::parameter::stepperSpeedControlPeriodMs[i];
    uint32_t stepsPerRev = ros::parameter::stepperStepsPerRev[i];

    Stepper stepper(pinout::armStepperPul[i], pinout::armStepperDir[i],
        stepsPerRev,speedControlPeriodMs);

    rover_arm_interfaces__msg__ArmStepper armStepperMsgReceived{};
    rover_arm_interfaces__msg__StepperFeedback stepperFeedbackSent{};

    
    while (true) {
        if (xQueueReceive(queue::armStepperQueues[i],
                &armStepperMsgReceived,
                ros::parameter::stepperTimeoutMs) != pdTRUE) {
            stepper.enable(false);
        } else {
            stepper.startMotion(static_cast<int32_t>(armStepperMsgReceived.target_pos_steps),
                ros::parameter::stepperMaxAccel[i], 500, true);
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
        stepperFeedbackSent.speed_steps_sec = stepper.getActualSpeed();
        xQueueOverwrite(queue::stepperFeedbackQueues[i], &stepperFeedbackSent);
    }
}

void microRosTask(void* arg) {
    // Suspend the FreeRTOS scheduler since the MicroROS initialization is not
    // thread safe.
    vTaskSuspendAll();

    // MicroRos boiler plate.
    rcl_allocator_t allocator = rcl_get_default_allocator();

    rclc_support_t support{};
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node = rcl_get_zero_initialized_node();
    rclc_node_init_default(&node, "pico_node", "arm", &support);

    // Create the MicroROS motor feedback publishers
    ros::createMotorFeedbackPublishers(&node);

    // Create the MicroROS gripper motor subscribers.
    auto gripperMsgType = ROSIDL_GET_MSG_TYPE_SUPPORT(rover_drive_interfaces, msg, MotorDrive);
    constexpr etl::array<etl::string_view, ros::gripperMsgs.size()> gripperSubscriberNames{
        "gripper_subscriber_0", "gripper_subscriber_1", "gripper_subscriber_2"
    };
    etl::array<ros::Subscriber, 3> gripperSubscribers{
        ros::Subscriber(&node, gripperSubscriberNames[0], gripperMsgType),
        ros::Subscriber(&node, gripperSubscriberNames[1], gripperMsgType),
        ros::Subscriber(&node, gripperSubscriberNames[2], gripperMsgType)
    };

    // Create the MicroROS arm stepper subscribers.
    auto armStepperMsgType = ROSIDL_GET_MSG_TYPE_SUPPORT(rover_drive_interfaces, msg, MotorDrive);
    constexpr etl::array<etl::string_view, ros::armStepperMsgs.size()> armStepperSubscriberNames{
        "arm_gripper_subscriber_0", "arm_gripper_subscriber_1", "arm_gripper_subscriber_2"
    };
    etl::array<ros::Subscriber, 3> armStepperSubscribers{
        ros::Subscriber(&node, armStepperSubscriberNames[0], armStepperMsgType),
        ros::Subscriber(&node, armStepperSubscriberNames[1], armStepperMsgType),
        ros::Subscriber(&node, armStepperSubscriberNames[2], armStepperMsgType)
    };

    // Create MicroROS timer that will publish the feedback messeages
    // We use a MicroROS task for publishing instead of publishing directly in
    // FreeRTOS tasks as I had problem with random crashes when publishing from
    // different cores at the same time.
    rcl_timer_t publisherTimer = rcl_get_zero_initialized_timer();
    publisherTimer = rcl_get_zero_initialized_timer();
    rclc_timer_init_default(&publisherTimer, &support,
        RCL_MS_TO_NS(ros::parameter::feedbackPeriodMs), ros::publisherTimerCallback);

    // Create the executor responsible for all the subscribers and the timer.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 5, &allocator);
    // Add the subscribers and the timer to the executor.
    for (int i = 0; i < gripperSubscribers.size(); i++) {
        gripperSubscribers[i].addToExecutor(&executor, &ros::gripperMsgs[i],
            ros::gripperSubscriberCallback, queue::gripperMotorQueues[i], ON_NEW_DATA);
        armStepperSubscribers[i].addToExecutor(&executor, &ros::armStepperMsgs[i],
            ros::armStepperSubscriberCallback, queue::armStepperQueues[i], ON_NEW_DATA);
    }
    rclc_executor_add_timer(&executor, &publisherTimer);

    // Create the parameter server and a seperate executor for the parameter
    // server.
    // We need the second executor since we have more handles than the maximum
    // executor handle.
    ros::parameter::Server paramServer(&node, true, 18, true, false);
    rclc_executor_t paramServerExecutor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(
        &paramServerExecutor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, &allocator);
    paramServer.addToExecutor(&paramServerExecutor);
    paramServer.initParameters();

    // Create the FreeRTOS motor tasks that will run on both cores.
    freertos::createMotorTasks();
    // Resume the scheduler since the initialization is complete.
    xTaskResumeAll();
    // Set the watchdog timer that will reset microcontroller if it is not updated
    // within set time period.
    watchdog_enable(ros::parameter::feedbackPeriodMs * 10, true);
    while (true) {
        // Spin the executors to check if there are new subscriber messeages or
        // parameter server requests.
        const auto executorResult = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        const auto paramExecutorResult =
            rclc_executor_spin_some(&paramServerExecutor, RCL_MS_TO_NS(1));
        // Update the watchdog
        watchdog_update();
        // Delay the tasks to free the core for other tasks.
        // TODO add parameter to control executor period.
        vTaskDelay(pdMS_TO_TICKS(
            ros::parameter::feedbackPeriodMs > 50 ? 50 : ros::parameter::feedbackPeriodMs));
    }
}
} // namespace task

} // namespace freertos