cmake_minimum_required(VERSION 3.16)
project(SimpleReedsSheppCarPlanner)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED base)

include_directories( src/include )

add_library(simplereedssheppcarplanner SHARED
        src/include/MultipleCircleStateValidityChecker.h
        src/MultipleCircleStateValidityChecker.cpp
        src/MultipleCircleReedsSheppCarPlanner.cpp)
target_link_libraries(simplereedssheppcarplanner PRIVATE ompl::ompl)

add_library(cachingreedssheppcarplanner SHARED
        src/include/MultipleCircleStateValidityChecker.h
        src/MultipleCircleStateValidityChecker.cpp
        src/CachingReedsSheppCarPlanner.cpp)
target_link_libraries(cachingreedssheppcarplanner PRIVATE ompl::ompl)