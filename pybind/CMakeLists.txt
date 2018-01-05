cmake_minimum_required(VERSION 2.8.4)
project(pybind_example)

set(CMAKE_CXX_STANDARD 11)
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

# Include header files
include_directories(include)

# setup pybind11
set(PYBIND11_CPP_STANDARD -std=c++11)
set(PYBIND11_PYTHON_VERSION 2.7)
add_subdirectory(${PYBIND_DIR}/src)
pybind11_add_module(pybind_example src/main.cpp)