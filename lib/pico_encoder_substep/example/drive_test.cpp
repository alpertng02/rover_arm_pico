#include "pico/stdlib.h"
#include "motor.hpp"
#include "encoder_substep.hpp"

#include <stdio.h>

float get_rev_per_sec(int enc_speed, int encoder_pulse_per_rev, float gear_ratio) {
    return static_cast<float>(enc_speed) / 1000 / gear_ratio / (encoder_pulse_per_rev / 4);
}

void set_speed(Motor* motors, float a, float b, float c, float d) {
    motors[0].setSpeedPercent(a);
    motors[1].setSpeedPercent(b);
    motors[2].setSpeedPercent(c);
    motors[3].setSpeedPercent(d);
}

enum Pos {
    fl = 0,
    fr,
    bl,
    br
};

int main() {

    stdio_init_all();
    sleep_ms(2500);

    pio_add_program(pio0, &quadrature_encoder_substep_program);
 
    EncoderSubstep encoders[4] {
        { pio0, 0, 10 },
        { pio0, 1, 12 },
        { pio0, 2, 14 },
        { pio0, 3, 20 }
    };

    Motor motors[4] {
        { 2, 3 },
        { 4, 5 },
        { 6, 7 },
        { 8, 9 }
    };

    printf("Encoder Created!\n");

    const char* strings[4] { "FL", "FR", "BL", "BR" };
    int speed_percent = 0;

    // for (int i = 20; i <= 40; i += 10) {
    //     set_speed(motors, i, i, i, i);
    //     sleep_ms(250);
    // }
    // set_speed(motors, 50, 50, 50, 50);
    // sleep_ms(5000);

    // set_speed(motors, 50, 0, 50, 0);
    // sleep_ms(250);

    // set_speed(motors, 50, -25, 50, -25);
    // sleep_ms(750);

    // set_speed(motors, 50, 0, 50, 0);
    // sleep_ms(250);

    // set_speed(motors, 45, 45, 45, 45);

    // sleep_ms(3500);

    // set_speed(motors, 0, 0, 0, 0);
    // sleep_ms(500);
    // set_speed(motors, -50, 50, -50, 50);
    // sleep_ms(2750);

    // set_speed(motors, 0, 50, 0, 50);
    // sleep_ms(500);

    // set_speed(motors, 45, 45, 45, 45);
    // sleep_ms(3500);

    // set_speed(motors, 0, 50, 0, 50);
    // sleep_ms(2250);

    // set_speed(motors, 80, 80, 80, 80);
    // sleep_ms(2000);

    // set_speed(motors, 0, 0, 0, 0);

    while (true) {
        int motor = 0;
        float motor_speed = 0;
        scanf("%d %f", &motor, &motor_speed);

        motors[motor].setSpeedPercent(motor_speed);
        sleep_ms(10);

    }
    while (true) {
        tight_loop_contents();
    }



    return 0;
}