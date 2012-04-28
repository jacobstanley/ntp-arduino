# CMake Configuration

cmake_minimum_required(VERSION 2.8)

set(CMAKE_MODULE_PATH    ${CMAKE_SOURCE_DIR}/cmake/modules)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Arduino.cmake)

find_package(Arduino 1.0 REQUIRED)

# Project Configuration

project(ntp C CXX)
set(FIRMWARE_NAME ntp)

set(${FIRMWARE_NAME}_BOARD atmega328)
set(${FIRMWARE_NAME}_SRCS  src/main.cpp)
set(${FIRMWARE_NAME}_PORT  /dev/ttyUSB0)

generate_arduino_firmware(${FIRMWARE_NAME})
