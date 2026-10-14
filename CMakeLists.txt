cmake_minimum_required(VERSION 3.16)

project(TrackGuard VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "Build TrackGuard tests" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(trackguard
    src/main.cpp
)

target_include_directories(trackguard
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(trackguard
    PRIVATE
        Eigen3::Eigen
)

target_compile_features(trackguard PRIVATE cxx_std_17)

set_target_properties(trackguard PROPERTIES
    OUTPUT_NAME trackguard
)

if(BUILD_TESTS)
    enable_testing()
    message(STATUS "BUILD_TESTS=ON: test targets will be added in Day 4+")
endif()