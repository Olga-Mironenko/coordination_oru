cmake_minimum_required(VERSION 3.16)
project(SimpleReedsSheppCarPlanner)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED base)

include_directories(src/include)

file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

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

add_executable(Point2DPlanning
        src/Point2DPlanning.cpp
        src/PRMcustom.cpp
        src/include/PRMcustom.h
        src/include/Conditions.h
        src/include/ConditionsPPM.h
        src/include/PathFinder.h)
target_link_libraries(Point2DPlanning PRIVATE ompl::ompl)