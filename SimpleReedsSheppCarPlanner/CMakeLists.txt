cmake_minimum_required(VERSION 3.29)
project(SimpleReedsSheppCarPlanner)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED base)

include_directories( src/include )

add_library(simplereedssheppcarplanner SHARED
        src/include/MultipleCircleStateValidityChecker.h
        src/MultipleCircleStateValidityChecker.cpp
        src/MultipleCircleReedsSheppCarPlanner.cpp)

target_link_libraries(simplereedssheppcarplanner PRIVATE ompl::ompl)