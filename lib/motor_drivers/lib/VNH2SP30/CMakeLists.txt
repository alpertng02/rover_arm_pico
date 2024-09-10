# Create an interface library for pico-stepper
add_library(pico_VNH2SP30 INTERFACE)

# Specify the source files for the pico-stepper library
target_sources(pico_VNH2SP30 INTERFACE 
    VNH2SP30.cpp
)

# Link the necessary libraries to pico-stepper
target_link_libraries(pico_VNH2SP30
    INTERFACE 
        hardware_pwm
        hardware_clocks
        hardware_gpio
        hardware_adc
        pico_IMotor
        pico_PwmController
)

# Include the current source directory for header files
target_include_directories(pico_VNH2SP30
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
)