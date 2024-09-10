#include "pico/stdlib.h"
#include "BTS7960.hpp"

#include <cstdio>

int main() {
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    Motor::BTS7960 motor(2, 3);
    int dir = 1;
    Motor::BTS7960::Alarm alarm = Motor::BTS7960::noAlarm;
    while ((alarm = motor.checkAlarm()) == Motor::BTS7960::noAlarm) {
        for (int32_t i = 0; i <= 10000; i++) {
            motor.setSpeed(i * dir);
            sleep_us(100);
        }
        sleep_ms(100);
        dir = -dir;
    }
    printf("Alarm %d\n", alarm);

    return 0;
}