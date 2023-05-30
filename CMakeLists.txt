cmake_minimum_required(VERSION 3.5)
project(cyber_dog_motor_sdk)

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-O3")

FIND_PACKAGE(lcm REQUIRED)

file(GLOB_RECURSE sources src/*.c  src/*.cpp)

# generate cyber_dog_motor_sdk library
add_library(cyber_dog_motor_sdk SHARED ${sources})
target_include_directories(cyber_dog_motor_sdk PUBLIC
                    "include"
)
target_link_libraries(cyber_dog_motor_sdk
  pthread
  lcm
)

# compile use code
add_executable(example_motor_ctrl src/example_motor_ctrl.cpp)
target_link_libraries(example_motor_ctrl cyber_dog_motor_sdk)