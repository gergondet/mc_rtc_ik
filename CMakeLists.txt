cmake_minimum_required(VERSION 3.10)

project(mc-rtc-ik LANGUAGES CXX VERSION 1.0.0)

find_package(benchmark REQUIRED)
find_package(mc_rtc REQUIRED)

add_library(mc-rtc-ik-helpers OBJECT src/helpers.h src/helpers.cpp)
target_link_libraries(mc-rtc-ik-helpers PUBLIC mc_rtc::mc_control)

add_executable(mc-rtc-ik src/main.cpp)
target_link_libraries(mc-rtc-ik PUBLIC mc-rtc-ik-helpers)

add_executable(mc-rtc-ik-tvm src/main-tvm.cpp)
target_link_libraries(mc-rtc-ik-tvm PUBLIC mc-rtc-ik-helpers)

add_executable(mc-rtc-ik-velocity src/main-velocity.cpp)
target_link_libraries(mc-rtc-ik-velocity PUBLIC mc-rtc-ik-helpers)

add_executable(bench-mc-rtc-ik src/bench.cpp)
target_link_libraries(bench-mc-rtc-ik PUBLIC mc-rtc-ik-helpers benchmark::benchmark)
