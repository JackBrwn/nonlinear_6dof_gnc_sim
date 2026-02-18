cmake_minimum_required(VERSION 3.10)

project(RigidBodySim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(main
    tests/test_RigidBodyData.cpp
    src/RigidBody.cpp)

set_target_properties(main PROPERTIES WIN32_EXECUTABLE OFF)