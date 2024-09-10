# Create an interface library for pico-stepper
add_library(pico_IMotor INTERFACE)

# Specify the source files for the pico-stepper library
target_sources(pico_IMotor INTERFACE 
    ${CMAKE_CURRENT_LIST_DIR}/IMotor.hpp
)

# Include the current source directory for header files
target_include_directories(pico_IMotor
    INTERFACE 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
        $<INSTALL_INTERFACE:include>
)