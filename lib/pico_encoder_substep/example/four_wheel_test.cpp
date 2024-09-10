#include "pico/stdlib.h"
#include "motor.hpp"
#include "encoder_substep.hpp"

#include <stdio.h>

float get_rev_per_sec(int enc_speed, int encoder_pulse_per_rev, float gear_ratio) {
    return static_cast<float>(enc_speed) / 1000 / gear_ratio / (encoder_pulse_per_rev / 4);
}

enum Pos {
    fl = 0,
    fr,
    bl,
    br
};


#define CLAMP(x, upper, lower) (MIN((upper), MAX((x), (lower))))

constexpr float Kp { static_cast<float>(80.0f) / 15500 * 1.5f };
constexpr float Ki { Kp / 10.0f };
// constexpr float Kd { Kp / 50.0f };
constexpr float integral_boundary { 80.0f };


// bool encoder_reader(repeating_timer_t* timer) {
//     static EncoderSubstep encoders[4] {
//         { pio0, 0, 10 },
//         { pio0, 1, 12 },
//         { pio0, 2, 14 },
//         { pio0, 3, 20 }
//     };

//     static Motor motors[4] {
//         { 8, 9 },
//         { 2, 3 },
//         { 4, 5 },
//         { 7, 6 }
//     };

//     static float integral[4] {};
//     const char* strings[4] { "FL", "FR", "BL", "BR" };
//     const int motor_dirs[4] { -1, 1, -1, 1 };
//     for (int i = 0; i < 4; i++) {
//         const float target_rpm = ((float*) (timer->user_data))[i];
//         const float current_rpm = encoders[i].getSpeed() * motor_dirs[i] * (4 * 60) / 16000.0f;

//         const float error { static_cast<float>(target_rpm - current_rpm) };

//         const float proportional = Kp * error;

//         integral[i] = CLAMP(integral[i] + error * Ki, integral_boundary, -integral_boundary);
//         // integral[i] += error * Ki;
//         // const int32_t deltaT = static_cast<int32_t>(t->delay_us);
//         // const float derivative = Kd * (error - prev_error) / (deltaT);
//         // prev_error = error;

//         // const float motor_pwm = CLAMP(proportional + integral[i], 80.0f, -80.0f);
//         const float motor_pwm =target_rpm;
//         printf("M : %s, RPM : %.3f, PWM : %.3f\n",
//             strings[i], current_rpm, motor_pwm);
//         // printf("Target_RPM : %.3f,  Error : %.3f  Proportional : %.3f  Integral : %.3f\n",
//         //     target_rpm, error, proportional, integral[i]);

//         motors[i].setSpeedPercent(motor_pwm);

//     }
//     printf("\n");
//     return true;
// }

int main() {

    stdio_init_all();
    sleep_ms(2000);

    pio_add_program(pio0, &quadrature_encoder_substep_program);


    Motor motors[4] {
        { 2, 3 },
        { 4, 5 },
        { 6, 7 },
        { 8, 9 }
    };

    printf("Encoder Created!\n");

    sleep_ms(1000);
    repeating_timer timer {};
    float speed_percent[4] = {30.0f, 30.0f, 30.0f, 30.0f};
    // add_repeating_timer_ms(-40, encoder_reader, speed_percent, &timer);

    const char* strings[4] { "FL", "FR", "BL", "BR" };
    while (1) {

        printf("\nFL FR BL BR\n\n");
        scanf("%f %f %f %f", &speed_percent[0], &speed_percent[1], &speed_percent[2], &speed_percent[3]);
        for (int i = 0; i < 4; i++) {
            motors[i].setSpeedPercent(speed_percent[i]);
        }
        sleep_ms(10);
    }

    return 0;
}