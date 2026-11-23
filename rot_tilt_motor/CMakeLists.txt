cmake_minimum_required(VERSION 3.20)

# also look for boards in xray-f26/boards (where motorboard lives)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(rot_tilt_motor)

target_sources(app PRIVATE
    src/main.c
    src/rot_motor_test.c
)