cmake_minimum_required(VERSION 2.8.4)
project(http_image)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download CPR
#

set(CPR_DIR ${CMAKE_CURRENT_BINARY_DIR}/cpr-download)

# Download pybind11
configure_file(
  "${CMAKE_MODULE_PATH}/cpr.cmake.in"
  "${CPR_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${CPR_DIR} )
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${CPR_DIR})

# add the cpr CMake module to our current project
add_subdirectory(${CPR_DIR}/src ${CPR_DIR}/build EXCLUDE_FROM_ALL)

###############################################################################

# Include header files
include_directories(${CPR_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${CPR_LIBRARIES})
