cmake_minimum_required(VERSION 3.29)
project(ompl_test)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED base)

file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

add_executable(Point2DPlanning
        Point2DPlanning.cpp
        PRMcustom.cpp
        PRMcustom.h
        Conditions.h
        ConditionsPPM.h
        PathFinder.h)
target_link_libraries(Point2DPlanning PRIVATE ompl::ompl)

add_executable(PlannerData
        PlannerData.cpp
        PRMcustom.cpp
        PRMcustom.h)
target_link_libraries(PlannerData PRIVATE
        ompl::ompl
        Eigen3::Eigen
        Boost::filesystem
        Boost::system)

add_executable(memcmp_performance memcmp_performance.c)
add_executable(memory_difference memory_difference.c)
