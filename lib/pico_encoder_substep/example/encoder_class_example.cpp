#include "pico/stdlib.h"
#include "motor.hpp"
#include "encoder_substep.hpp"

#include <stdio.h>

int main() {

    stdio_init_all();
    sleep_ms(2000);

    // base pin to connect the A phase of the encoder. the B phase must be
    // connected to the next pin
    const uint PIN_A = 2;
    const uint motorL = 0;
    const uint motorR = 1;

    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    PIO pio = pio0;
    const uint sm = 0;

    EncoderSubstep encoder(pio0, 0, PIN_A);

    printf("Encoder Created!\n");

    Motor motor(motorL, motorR, 6, 7);
    motor.setSpeedPercent(95.0f);

    sleep_ms(3000);
    encoder.calibrate();

    while (1) {
        auto data { encoder.getAll() };
        // print out the result
        printf("%10d %10d %10d\n", data.speed_2_20, data.speed, data.step);

        // run at roughly 100Hz
        sleep_ms(10);
    }

    return 0;
}