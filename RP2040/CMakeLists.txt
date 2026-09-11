cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(robot_triathlon C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable(robot_triathlon
    robot.c
    hw.c
)

# Link against pico_stdlib for runtime, and hardware libraries
target_link_libraries(robot_triathlon
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_adc
    hardware_timer
)

# Enable standard library (math functions: sinf, cosf, sqrtf)
target_link_libraries(robot_triathlon
    m
)

# Enable USB output (optional, for debug printf)
pico_enable_stdio_usb(robot_triathlon 1)
pico_enable_stdio_uart(robot_triathlon 0)

# Generate UF2 file for drag-and-drop flashing
pico_add_extra_outputs(robot_triathlon)
