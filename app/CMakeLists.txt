cmake_minimum_required(VERSION 3.20)

# for global installs
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# populated the $PROJECT_NAME var and this is what the output files are named
project(firepod)

# lets the compiler find the .h files in src/
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# app is a predesignated target that zephyr uses
# (tilt motor + IMU verification; distance sensor commented out for this test)
target_sources(app PRIVATE
    src/main.c
    src/tilt_motor.c
    src/imu_tilt.c
    # src/distance_sensor.c
    # src/platform.c
    # src/VL53L4CD_api.c
    # src/VL53L4CD_calibration.c
    # src/imu.c
)