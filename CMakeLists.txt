cmake_minimum_required(VERSION 3.25)
project(auv_sim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(auv_sim src/main.cpp src/integrators.cpp src/rigid_body.cpp)
target_include_directories(auv_sim PRIVATE include)
target_compile_options(auv_sim PRIVATE -Wall -Wextra -Wpedantic)