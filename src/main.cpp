/**
 * @file main.cpp
 * @author Alper Tunga Güven (alpert.guven@gmail.com)
 * @brief Starting point of program.
 * @version 0.1
 * @date 2024-09-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <pico/stdlib.h>

extern "C" {

#include <FreeRTOS.h>
#include <task.h>

#include "pico_uart_transports.h"
#include <rmw_microros/rmw_microros.h>
}

#include "pinout.hpp"
#include "freertos.hpp"
#include <cstdio>

int main() {
    // Set the custom transport functions that MicroROS is going to use.
    rmw_uros_set_custom_transport(true, NULL, pico_serial_transport_open,
        pico_serial_transport_close, pico_serial_transport_write, pico_serial_transport_read);

    // Setup the onboard led.
    gpio_init(pinout::led);
    gpio_set_dir(pinout::led, GPIO_OUT);

    // Setup all the FreeRTOS queues that will transfer data across tasks in a
    // thread safe manner.
    freertos::createMsgQueues();

    // Create and start the main task off the program.
    freertos::createMicroRosTask();

    freertos::createGripperMotorTasks();
    freertos::createStepperMotorTasks();

    vTaskStartScheduler();

    // Code will never reach here.
    while (true) {}
    return 0;
}