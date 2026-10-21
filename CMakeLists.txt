cmake_minimum_required(VERSION 3.16)

project(TrackGuard VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "Build TrackGuard tests" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)

add_executable(trackguard
    src/main.cpp
    src/config/run_config.cpp
    src/replay/kitti_loader.cpp
    src/replay/replay_clock.cpp
)

target_include_directories(trackguard
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(trackguard
    PRIVATE
        Eigen3::Eigen
        yaml-cpp
)

target_compile_features(trackguard PRIVATE cxx_std_17)
target_compile_definitions(trackguard PRIVATE
    TRACKGUARD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

set_target_properties(trackguard PROPERTIES
    OUTPUT_NAME trackguard
)

if(BUILD_TESTS)
    enable_testing()
    find_package(GTest QUIET)
    if(NOT GTest_FOUND AND EXISTS "/usr/src/googletest/CMakeLists.txt")
        add_subdirectory(
            /usr/src/googletest
            ${CMAKE_CURRENT_BINARY_DIR}/googletest
            EXCLUDE_FROM_ALL
        )
    endif()
    if(TARGET GTest::gtest_main)
        set(TRACKGUARD_GTEST_MAIN_TARGET GTest::gtest_main)
    elseif(TARGET gtest_main)
        set(TRACKGUARD_GTEST_MAIN_TARGET gtest_main)
    else()
        message(FATAL_ERROR "GoogleTest is required when BUILD_TESTS=ON")
    endif()

    add_executable(trackguard_day3_tests
        tests/day3_tests.cpp
        src/config/run_config.cpp
    )
    target_include_directories(trackguard_day3_tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(trackguard_day3_tests
        PRIVATE
            yaml-cpp
    )
    target_compile_features(trackguard_day3_tests PRIVATE cxx_std_17)
    target_compile_definitions(trackguard_day3_tests PRIVATE
        TRACKGUARD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    add_test(NAME trackguard_day3_tests COMMAND trackguard_day3_tests)

    add_executable(trackguard_day4_tests
        tests/kitti_loader_tests.cpp
        src/replay/kitti_loader.cpp
    )
    target_include_directories(trackguard_day4_tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(trackguard_day4_tests
        PRIVATE
            ${TRACKGUARD_GTEST_MAIN_TARGET}
    )
    target_compile_features(trackguard_day4_tests PRIVATE cxx_std_17)
    include(GoogleTest)
    gtest_discover_tests(trackguard_day4_tests)

    add_executable(trackguard_day5_tests
        tests/replay_clock_tests.cpp
        src/replay/replay_clock.cpp
    )
    target_include_directories(trackguard_day5_tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(trackguard_day5_tests
        PRIVATE
            ${TRACKGUARD_GTEST_MAIN_TARGET}
    )
    target_compile_features(trackguard_day5_tests PRIVATE cxx_std_17)
    gtest_discover_tests(trackguard_day5_tests)
endif()