#include "tasks.hpp"
#include "freertos.hpp"
#include "parameters.hpp"
#include "publisher.hpp"
#include "queues.hpp"
#include "subscriber.hpp"

#include "BTS7960.hpp"
#include "encoder_substep.hpp"
#include "pinout.hpp"

#include <hardware/watchdog.h>

namespace freertos {

namespace task {

void motorTask(void* arg) {
    // Convert the index into size_t. This void* arg is the value we got from
    // indexes[i] in createMotorTasks()
    const size_t i = (size_t)(arg);

    // Create the motor and encoder classes.
    motor::BTS7960 motor(pinout::motorPwmL[i], pinout::motorPwmR[i]);
    encoder::EncoderSubstep encoder(
        pinout::encoderPio, pinout::encoderPioSm[i], pinout::encoderA[i]);

    // Create the ros messeages.
    rover_drive_interfaces__msg__MotorDrive driveMsgReceived{};
    rover_drive_interfaces__msg__MotorFeedback feedbackMsgSent{};

    // Integral and previous error to use for PID.
    float integral = 0.0f;
    float errorRpmPrev = 0.0f;

    absolute_time_t lastMsgReceivedTime = get_absolute_time();
    // Get the starting time of the task.
    TickType_t startTick{ xTaskGetTickCount() };
    while (true) {
        // If there is a new drive messeage available, receive it and update the
        // lastMsgReceivedTime variable. Otherwise, don't wait for new data and
        // continue.
        if (xQueueReceive(freertos::queue::driveQueues[i], &driveMsgReceived, 0) == pdTRUE) {
            lastMsgReceivedTime = get_absolute_time();
        }

        // Read the current RPM from encoders.
        feedbackMsgSent.encoder_rpm = encoder.getRpm();
        // Calculate the time since the last messeage was received.
        auto timeDiffMsg = absolute_time_diff_us(lastMsgReceivedTime, get_absolute_time()) / 1000;

        // If the motor current is over the maximum value or the timeout has been
        // reached, set the target RPM to 0 to turn the motors off.
        if (feedbackMsgSent.current >= ros::parameter::maxMotorCurrent ||
            timeDiffMsg >= ros::parameter::motorTimeoutMs) {
            driveMsgReceived.target_rpm = 0;
        }
        // If PID mode is open calculate the PID values.
        if (ros::parameter::motorPidMode) {
            const int32_t errorRpm = driveMsgReceived.target_rpm - feedbackMsgSent.encoder_rpm;

            const float proportional = errorRpm * ros::parameter::motorPidKd;
            integral += errorRpm * ros::parameter::motorPidKi;
            integral = etl::clamp(
                integral, -ros::parameter::maxMotorDutyCycle, ros::parameter::maxMotorDutyCycle);
            const float derivative = ros::parameter::motorPidKd * (errorRpm - errorRpmPrev) *
                                     ros::parameter::motorPidLoopPeriodMs / 1000.0f;

            feedbackMsgSent.dutycycle = proportional + integral + derivative;
        } else {
            // If PID mode is off, assume there is a linear relationship between the
            // motor dutycyle and RPM and use open loop control.
            feedbackMsgSent.dutycycle = driveMsgReceived.target_rpm *
                                        ros::parameter::maxMotorDutyCycleUpperConstraint /
                                        ros::parameter::maxMotorRpm;
        }
        // Clamp the dutycycle within the boundaries.
        feedbackMsgSent.dutycycle = etl::clamp(feedbackMsgSent.dutycycle,
            -ros::parameter::maxMotorDutyCycle,
            ros::parameter::maxMotorDutyCycle);
    
        // Imaginary current value to test current limiting property since we do
        // not have a current sensor yet.
        feedbackMsgSent.current =
            (feedbackMsgSent.dutycycle) * (ros::parameter::maxMotorCurrent - 5.0f) / 100.0f;

        // Set the dutycyle of the motors.
        motor.setSpeed(feedbackMsgSent.dutycycle);
        // Send the feedback messeage to the queue
        xQueueOverwrite(freertos::queue::publisherQueues[i], &feedbackMsgSent);
        // Delay the task by the amount set in motor_pid_loop_period_ms parameter.
        xTaskDelayUntil(&startTick, pdMS_TO_TICKS(ros::parameter::motorPidLoopPeriodMs));
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
    rclc_node_init_default(&node, "pico_node", "drive", &support);

    // Create the MicroROS motor feedback publishers
    ros::createMotorFeedbackPublishers(&node);

    // Create the MicroROS motor drive subscribers.
    constexpr etl::array<etl::string_view, 4> subscriberNames{ "pico_subscriber_0",
        "pico_subscriber_1", "pico_subscriber_2", "pico_subscriber_3" };
    auto subscriberMsgType = ROSIDL_GET_MSG_TYPE_SUPPORT(rover_drive_interfaces, msg, MotorDrive);
    etl::array<ros::Subscriber, 4> driveSubscribers{ ros::Subscriber(&node, subscriberNames[0],
                                                         subscriberMsgType),
        ros::Subscriber(&node, subscriberNames[1], subscriberMsgType),
        ros::Subscriber(&node, subscriberNames[2], subscriberMsgType),
        ros::Subscriber(&node, subscriberNames[3], subscriberMsgType) };

    // Create MicroROS timer that will publish the feedback messeages
    // We use a MicroROS task for publishing instead of publishing directly in
    // FreeRTOS tasks as I had problem with random crashes when publishing from
    // different cores at the same time.
    rcl_timer_t publisherTimer = rcl_get_zero_initialized_timer();
    publisherTimer = rcl_get_zero_initialized_timer();
    rclc_timer_init_default(&publisherTimer, &support,
        RCL_MS_TO_NS(ros::parameter::motorFeedbackPeriodMs), ros::publisherTimerCallback);

    // Create the executor responsible for all the subscribers and the timer.
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, 5, &allocator);
    // Add the subscribers and the timer to the executor.
    for (int i = 0; i < driveSubscribers.size(); i++) {
        driveSubscribers[i].addToExecutor(&executor, &ros::driveMsgs[i],
            ros::driveSubscriberCallback, queue::driveQueues[i], ON_NEW_DATA);
    }
    rclc_executor_add_timer(&executor, &publisherTimer);

    // Create the parameter server and a seperate executor for the parameter
    // server.
    // We need the second executor since we have more handles than the maximum
    // executor handle.
    ros::parameter::Server paramServer(&node, true, 10, true, false);
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
    watchdog_enable(ros::parameter::motorFeedbackPeriodMs * 10, true);
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
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
} // namespace task

} // namespace freertos