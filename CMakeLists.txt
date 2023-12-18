cmake_minimum_required(VERSION 3.10)

project(mc-rtc-ik LANGUAGES CXX VERSION 1.0.0)

find_package(mc_rtc REQUIRED)

add_executable(mc-rtc-ik src/main.cpp)
target_link_libraries(mc-rtc-ik PUBLIC mc_rtc::mc_control)
