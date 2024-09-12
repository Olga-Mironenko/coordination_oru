cmake_minimum_required(VERSION 2.8.9)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-m64")
SET(CMAKE_C_FLAGS "-m64")
SET(CMAKE_EXE_LINKER_FLAGS "-m64")
SET(CMAKE_MODULE_LINKER_FLAGS "-m64")
SET(CMAKE_SHARED_LINKER_FLAGS "-m64")
project (ompl-test)

find_package(ompl REQUIRED base)

file(TO_NATIVE_PATH "${CMAKE_SOURCE_DIR}/resources" TEST_RESOURCES_DIR)
string(REPLACE "\\" "\\\\" TEST_RESOURCES_DIR "${TEST_RESOURCES_DIR}")
add_definitions(-DTEST_RESOURCES_DIR=\"${TEST_RESOURCES_DIR}\")

macro(add_ompl_demo demo_name)
    add_executable(${ARGV})
    target_link_libraries(${demo_name} PRIVATE
        ompl::ompl
        Eigen3::Eigen
        Boost::filesystem
        Boost::system)
endmacro(add_ompl_demo)

add_ompl_demo(demo_Point2DPlanning Point2DPlanning.cpp)