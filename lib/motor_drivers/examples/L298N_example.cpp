#include "pico/stdlib.h"
#include "L298N.hpp"

#include <cstdio>

int main() {
    const uint LED_PIN = 25;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    Motor::L298N motor(2, 3, 4);

    int dir = 1;
    while (true) {
        for (int32_t i = 0; i <= 10000; i++) {
            motor.setSpeed(i * dir);
            sleep_us(100);
        }
        sleep_ms(100);
        dir = -dir;
    }
    return 0;
}