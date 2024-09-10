
# Add the executable for the stepper example
add_executable(BTS7960_example BTS7960_example.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(BTS7960_example 
    pico_stdlib
    pico_BTS7960
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(BTS7960_example)

# Enable USB output, disable UART output
pico_enable_stdio_usb(BTS7960_example 1)
pico_enable_stdio_uart(BTS7960_example 0)

#------------------------------------------------------------------

# Add the executable for the stepper example
add_executable(L298N_example L298N_example.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(L298N_example 
    pico_stdlib
    pico_L298N
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(L298N_example)

# Enable USB output, disable UART output
pico_enable_stdio_usb(L298N_example 1)
pico_enable_stdio_uart(L298N_example 0)

#------------------------------------------------------------------


# Add the executable for the stepper example
add_executable(VNH2SP30_example VNH2SP30_example.cpp)

# Link the necessary libraries to the stepper example
target_link_libraries(VNH2SP30_example 
    pico_stdlib
    pico_VNH2SP30
)

# Add extra outputs for the stepper example
pico_add_extra_outputs(VNH2SP30_example)

# Enable USB output, disable UART output
pico_enable_stdio_usb(VNH2SP30_example 1)
pico_enable_stdio_uart(VNH2SP30_example 0)