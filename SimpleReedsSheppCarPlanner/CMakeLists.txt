cmake_minimum_required(VERSION 3.16)
project(SimpleReedsSheppCarPlanner)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE Debug)

set(OUTPUT_DIR "${CMAKE_BINARY_DIR}/build-host" CACHE PATH "Output directory")

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
set_target_properties(simplereedssheppcarplanner PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
)

add_library(cachingplanner SHARED
        src/CachingPlanner.cpp
        src/Footprint.cpp
        src/PRMcustom.cpp
)
target_link_libraries(cachingplanner PRIVATE ompl::ompl)
set_target_properties(cachingplanner PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}"
)

add_executable(Point2DPlanning
        src/Point2DPlanning.cpp
        src/PRMcustom.cpp
        src/include/PRMcustom.h
        src/include/Conditions.h
        src/include/ConditionsPPM.h
        src/include/ConditionsOccupancy.h
        src/include/PathFinder.h
        src/include/Footprint.h
        src/Footprint.cpp
)
target_link_libraries(Point2DPlanning PRIVATE ompl::ompl)