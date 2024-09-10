#include "pico/stdlib.h"
#include "motor.hpp"
#include "encoder_substep.h"

#include <stdio.h>

int main() {

    stdio_init_all();
    sleep_ms(2000);


    substep_state_t state;

    // base pin to connect the A phase of the encoder. the B phase must be
    // connected to the next pin
    const uint PIN_A = 2;
    const uint motorL = 0;
    const uint motorR = 1;

    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    PIO pio = pio0;
    const uint sm = 0;

    pio_add_program(pio, &quadrature_encoder_substep_program);
    substep_init_state(pio, sm, PIN_A, &state);
    printf("Encoder Created!\n");

    Motor motor(motorL, motorR, 6, 7);
    motor.setSpeedPercent(95.0f);

    sleep_ms(3000);
    substep_phases_t substep_phases = {
        .first = 0,
        .second = 0,
        .third = 0
    };
    // substep_phases = substep_calibrate_phases(pio, sm);
    if (substep_phases.first == 0) {
        substep_set_calibration_data(&state, 64, 128, 192);
    } else {
        substep_set_calibration_data(&state, substep_phases.first, substep_phases.second, substep_phases.third);
    }

    while (1) {
        // read the PIO and update the state data
        // print out the result
        printf("%10d %10d %10d\n", state.position, state.speed, state.raw_step);

        // run at roughly 100Hz
        sleep_ms(10);
    }

    return 0;
}