cmake_minimum_required(VERSION 3.20)

project(flight_control
  VERSION 0.1.0
  DESCRIPTION "Demo quadrotor flight-control library and simulator"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

option(FC_ENABLE_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

if(FC_ENABLE_SANITIZERS)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

find_package(Threads REQUIRED)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
  DOWNLOAD_EXTRACT_TIMESTAMP ON)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

function(fc_warnings target)
  target_compile_options(${target} PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Werror
    -Wshadow
    -Wconversion
    -Wdouble-promotion)
endfunction()

add_library(fc
  src/pid.cpp
  src/controllers.cpp
  src/mixer.cpp
  src/complementary_filter.cpp
  src/attitude_ekf.cpp
  src/altitude_kf.cpp
  src/fixed_rate_loop.cpp
  src/quad_sim.cpp
  src/flight_controller.cpp
  src/scenario.cpp
  src/csv_logger.cpp)
target_include_directories(fc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(fc PUBLIC cxx_std_20)
target_link_libraries(fc PUBLIC Threads::Threads)
fc_warnings(fc)

add_executable(fc_sim_demo apps/sim_demo.cpp)
target_link_libraries(fc_sim_demo PRIVATE fc)
fc_warnings(fc_sim_demo)

enable_testing()
include(GoogleTest)

add_executable(fc_tests
  tests/test_math.cpp
  tests/test_pid.cpp
  tests/test_mixer_and_control.cpp
  tests/test_estimation.cpp
  tests/test_ring_and_sim.cpp
  tests/test_closed_loop.cpp)
target_link_libraries(fc_tests PRIVATE fc GTest::gtest_main)
fc_warnings(fc_tests)
gtest_discover_tests(fc_tests)
