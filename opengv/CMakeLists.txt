cmake_minimum_required(VERSION 2.8.4)
project(opengvtest)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# Find dependent libraries
find_package(Eigen3 REQUIRED)
find_package(opengv REQUIRED)

OPTION(BUILD_SHARED_LIBS "Build shared libraries" OFF)
OPTION(BUILD_POSITION_INDEPENDENT_CODE "Build position independent code (-fPIC)" ON)
  add_definitions (
    -O3
    -Wall
    -Wextra
    #-Werror
    -Wwrite-strings
    -Wno-unused-parameter
    -fno-strict-aliasing
    -fPIC
  )

include_directories(${EIGEN3_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(arunsac src/arunsac.cpp)
target_link_libraries(arunsac opengv)

add_executable(test_point_cloud_sac src/test_point_cloud_sac.cpp)
target_link_libraries(test_point_cloud_sac opengv)

set_target_properties(
    test_point_cloud_sac
    arunsac
    
    PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    DEBUG_POSTFIX d)
