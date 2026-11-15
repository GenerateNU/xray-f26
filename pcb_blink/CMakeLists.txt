cmake_minimum_required(VERSION 3.20)

# Tell Zephyr to also look for boards in xray-f26/boards (where motorboard lives)
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(pcb_blink)

# Minimal app to prove the custom STM32G491ME PCB can be flashed and runs
target_sources(app PRIVATE Src/main.c)
