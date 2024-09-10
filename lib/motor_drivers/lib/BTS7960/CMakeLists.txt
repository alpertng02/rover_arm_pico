# Create an interface library for pico-stepper
add_library(pico_BTS7960 INTERFACE)

# Specify the source files for the pico-stepper library
target_sources(pico_BTS7960 INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/BTS7960.cpp
)

# Link the necessary libraries to pico-stepper
target_link_libraries(pico_BTS7960
    INTERFACE 
        hardware_pwm
        hardware_clocks
        hardware_gpio
        pico_IMotor
        pico_PwmController
)

# Include the current source directory for header files
target_include_directories(pico_BTS7960
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        $<INSTALL_INTERFACE:include>
)