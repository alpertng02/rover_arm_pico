# Rover Drive Pico

This is the embedded firmware for Cengaver Rover's drive system using Raspberry Pi Pico.

## Features

- Control of all motors invidiually.
- Getting feedback at a specified rate about the dutycycle of the motor driver and RPM measured through the encoder.
- If a current sensor is added, ability to get feedback about motor current consumption.
- Through use of ROS parameters, various settings can be changed during runtime such as: 
    1. Ability to set maximum dutycycle for the motors through "max_motor_dutycyle" param.
    2. Ability to set the motor's maximum RPM through "max_motor_rpm" parameters.
    3. Ability to turn PID control OFF and ON through "motor_pid_mode" parameters.
    4. Ability to tune PID control using "motor_pid_kp", "motor_pid_ki" and "motor_pid_kd" parameters.
    5. Ability to change the rate of motor feedback to the ROS2 computer through "motor_feedback_period_ms" parameter.
    6. Ability to change the PID loop period through "motor_pid_loop_period_ms" parameter.
    7. Ability to set timeout duration to turn motors off when ROS2 communication stops through "motor_timeout_ms" parameter. 
    8. Ability to set the maximum motor current that will turn off the motors automatically through "motor_max_current" parameter.

## To build the firmware.

To build the firmware for Raspberry Pi Pico project, follow these steps:

### Setup Pico-SDK

#### Recommended:
- Install the [Raspberry Pi Pico Visual Studio Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) in VS Code.

#### Linux:
1. Install CMake (at least version 3.13), and a GCC cross compiler
```
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```
2.  Clone the Pico-SDK repo
```
git clone --recursive https://github.com/raspberrypi/pico-sdk.git 
```
3. Set the PICO_SDK_PATH environment variable
```
export PICO_SDK_PATH=/directory/to/your/sdk
```

#### Windows:
- Follow the steps in [Raspberry Pi Pico Windows installer](https://www.raspberrypi.com/news/raspberry-pi-pico-windows-installer/).

### To Build Project

1. Clone the repository with submodules: `git clone --recursive https://github.com/your-username/rover-drive-pico.git` 
2. Setup a CMake build directory.
      For example, if not using an IDE:
      ```
      $ mkdir build
      $ cd build
      $ cmake -DCMAKE_BUILD_TYPE=Release ..
      $ cmake --build .
      ```   

## Contributing

Contributions from Cengaver Rover team members are allowed.

## License

This project is licensed under the Apache 2.0 License. See the [LICENSE](LICENSE) file for more information.
