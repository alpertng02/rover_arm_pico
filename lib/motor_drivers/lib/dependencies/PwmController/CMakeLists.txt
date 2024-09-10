# Create an interface library for pico-stepper
add_library(pico_PwmController INTERFACE)

# Specify the source files for the pico-stepper library
target_sources(pico_PwmController INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/PwmController.cpp
)

# Link the necessary libraries to pico-stepper
target_link_libraries(pico_PwmController
    INTERFACE 
        hardware_gpio
        hardware_pwm
        hardware_clocks
)

# Include the current source directory for header files
target_include_directories(pico_PwmController
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        $<INSTALL_INTERFACE:include>
)