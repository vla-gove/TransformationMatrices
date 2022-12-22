cmake_minimum_required(VERSION 3.23)
project(6DOF_Robot C)

set(CMAKE_C_STANDARD 99)

add_executable(6DOF_Robot TranformationMatrices.c)
