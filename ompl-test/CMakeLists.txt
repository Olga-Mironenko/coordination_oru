cmake_minimum_required(VERSION 3.29)
project(ompl_test)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED base)

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
