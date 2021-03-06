cmake_minimum_required(VERSION 3.1)
project(pybind_example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

# Setup cmake paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#################################
#   Download pybind11 Project   #
#################################

set(PYBIND_DIR ${CMAKE_CURRENT_BINARY_DIR}/pybind11-download)

# Download pybind11
configure_file(
  "${CMAKE_MODULE_PATH}/pybind11.cmake.in"
  "${PYBIND_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${PYBIND_DIR} )
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${PYBIND_DIR})

#################################

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

# Include header files
include_directories(include ${EIGEN3_INCLUDE_DIRS})

# setup pybind11
# set(PYBIND11_CPP_STANDARD -std=c++11)
# set(PYBIND11_PYTHON_VERSION 3)
add_subdirectory(${PYBIND_DIR}/src)
pybind11_add_module(pybind_example src/main.cpp src/eigen.cpp src/smartptrs.cpp)