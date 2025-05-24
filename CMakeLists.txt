cmake_minimum_required(VERSION 3.31)
project(exp)

set(CMAKE_CXX_STANDARD 20)

add_executable(exp src/main.cpp
        src/robot.h
        src/grid.h
        src/simulation.h
        src/visual.h)
