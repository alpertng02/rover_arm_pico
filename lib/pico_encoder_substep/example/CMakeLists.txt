add_subdirectory(motor)

# add_executable(encoder_test encoder_example.cpp)

# target_link_libraries(encoder_test
#     pico_stdlib
#     pico_motor
#     pico_encoder_substep
# )

# pico_enable_stdio_usb(encoder_test 1)
# pico_enable_stdio_uart(encoder_test 0)

# pico_add_extra_outputs(encoder_test)    

# # -----------------------------------------------------------------

# add_executable(encoder_class_test encoder_class_example.cpp)

# target_link_libraries(encoder_class_test
#     pico_stdlib
#     pico_motor
#     pico_encoder_substep_class
# )

# pico_enable_stdio_usb(encoder_class_test 1)
# pico_enable_stdio_uart(encoder_class_test 0)

# pico_add_extra_outputs(encoder_class_test)    

# -----------------------------------------------------------------

add_executable(four_wheel_test four_wheel_test.cpp)

target_link_libraries(four_wheel_test
    pico_stdlib
    pico_motor
    pico_encoder_substep_class
)

pico_enable_stdio_usb(four_wheel_test 1)
pico_enable_stdio_uart(four_wheel_test 0)

pico_add_extra_outputs(four_wheel_test)    


# -----------------------------------------------------------------

add_executable(drive_test drive_test.cpp)

target_link_libraries(drive_test
    pico_stdlib
    pico_motor
    pico_encoder_substep_class
)

pico_enable_stdio_usb(drive_test 1)
pico_enable_stdio_uart(drive_test 0)

pico_add_extra_outputs(drive_test)    