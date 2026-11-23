cmake_minimum_required(VERSION 3.20)

# for global installs
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})


project(firepod)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)


target_sources(app PRIVATE
    src/main.c
    src/distance_sensor.c
    src/platform.c
    src/VL53L4CD_api.c
    src/VL53L4CD_calibration.c
)