cmake_minimum_required(VERSION 3.13)

# Set the name of the program
set(NAME rover_drive_pico) 

set(FREERTOS_KERNEL_PATH ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel)

include(pico_sdk_import.cmake)
include(FreeRTOS_Kernel_import.cmake)

# set language standards
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_compile_definitions( 
    PLATFORM_NAME_FREERTOS
)

#include microROS
SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/uros_ws/firmware/build" CACHE STRING "Course Common Lib")
include(micro_ros.cmake)

#include ETL
set(GIT_DIR_LOOKUP_POLICY ALLOW_LOOKING_ABOVE_CMAKE_SOURCE_DIR)
add_subdirectory(lib/etlcpp)

#include libs
add_subdirectory(lib/motor_drivers/lib)
add_subdirectory(lib/pico_encoder_substep/lib)

#Add main source directory
add_subdirectory(src)

#Set up files for the release packages
install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
