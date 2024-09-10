# Create an interface library for pico-stepper
add_library(pico_L298N INTERFACE)

# Specify the source files for the pico-stepper library
target_sources(pico_L298N INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/L298N.cpp
)

# Link the necessary libraries to pico-stepper
target_link_libraries(pico_L298N
    INTERFACE 
        hardware_pwm
        hardware_clocks
        hardware_gpio
        pico_IMotor
        pico_PwmController
)

# Include the current source directory for header files
target_include_directories(pico_L298N
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        $<INSTALL_INTERFACE:include>
)