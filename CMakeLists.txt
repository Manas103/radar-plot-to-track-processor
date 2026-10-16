cmake_minimum_required(VERSION 3.16)
project(radar_plot_to_track_processor CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra)

option(ENABLE_ASAN "Build with AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(ENABLE_TSAN "Build with ThreadSanitizer" OFF)
option(BUILD_TESTS "Build the GoogleTest suite" ON)

if(ENABLE_ASAN)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1)
  add_link_options(-fsanitize=address,undefined)
endif()

if(ENABLE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g -O1)
  add_link_options(-fsanitize=thread)
endif()

find_package(Threads REQUIRED)

add_library(radar_core STATIC
  src/kalman.cpp
  src/assignment.cpp
  src/tracker.cpp
  src/detection_sim.cpp
  src/replay.cpp
  src/sha256.cpp
)
target_include_directories(radar_core PUBLIC include)
target_link_libraries(radar_core PUBLIC Threads::Threads)

add_executable(radar_cli src/main.cpp)
target_link_libraries(radar_cli PRIVATE radar_core)

add_library(autolabel_core STATIC
  src/autolabel_types.cpp
  src/autolabel_scenario_gen.cpp
  src/autolabel_grid_prefilter.cpp
  src/autolabel_predicates.cpp
  src/autolabel_miner.cpp
)
target_include_directories(autolabel_core PUBLIC include)
target_link_libraries(autolabel_core PUBLIC Threads::Threads)

add_executable(autolabel_cli src/autolabel_main.cpp)
target_link_libraries(autolabel_cli PRIVATE autolabel_core radar_core)

if(BUILD_TESTS)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.zip
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(radar_tests
    tests/test_linalg.cpp
    tests/test_kalman.cpp
    tests/test_gating.cpp
    tests/test_assignment.cpp
    tests/test_ring.cpp
    tests/test_tracker.cpp
    tests/test_replay.cpp
  )
  target_link_libraries(radar_tests PRIVATE radar_core gtest_main)

  add_executable(autolabel_tests
    tests/test_autolabel_predicates.cpp
    tests/test_autolabel_prefilter.cpp
    tests/test_autolabel_miner.cpp
  )
  target_link_libraries(autolabel_tests PRIVATE autolabel_core gtest_main)

  include(GoogleTest)
  if(NOT ENABLE_TSAN)
    # Under ENABLE_TSAN, CMake's own test-discovery step (which runs the
    # binary once at build time to list its tests) hits ThreadSanitizer's
    # ASLR incompatibility on WSL2 before setarch -R has a chance to apply.
    # The TSan build is still exercised, just by invoking the binary
    # directly (with setarch -R) instead of through ctest.
    gtest_discover_tests(radar_tests)
    gtest_discover_tests(autolabel_tests)
  else()
    add_test(NAME radar_tests COMMAND radar_tests)
    add_test(NAME autolabel_tests COMMAND autolabel_tests)
  endif()
endif()
