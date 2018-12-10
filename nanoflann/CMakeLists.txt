cmake_minimum_required(VERSION 3.2)
project(nanoflann)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download nanoflann
#

set(NANOFLANN_DIR ${CMAKE_CURRENT_BINARY_DIR}/nanoflann-download)

# Download nanoflann
configure_file(
  "${CMAKE_MODULE_PATH}/nanoflann.cmake.in"
  "${NANOFLANN_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${NANOFLANN_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${NANOFLANN_DIR})

set(NANOFLANN_INCLUDE_DIRS ${NANOFLANN_DIR}/src/include/)

###############################################################################

# Include header files
message("Nanoflann: ${NANOFLANN_INCLUDE_DIRS}")
include_directories(src ${NANOFLANN_INCLUDE_DIRS})


# Create the executables and link them to libs
add_executable(pointcloud_example src/pointcloud_example.cpp)
target_link_libraries(pointcloud_example)
