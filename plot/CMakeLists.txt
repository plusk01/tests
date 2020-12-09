cmake_minimum_required(VERSION 3.10)
project(plotex)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

# Setup cmake paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#################################
#     Download plot library     #
#################################

set(PLOT_DIR ${CMAKE_CURRENT_BINARY_DIR}/plot-download)

# don't build plot examples
set(BUILD_EXAMPLES OFF CACHE BOOL "")

# Download plot
configure_file(
  "${CMAKE_MODULE_PATH}/plot.cmake.in"
  "${PLOT_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${PLOT_DIR} )
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${PLOT_DIR})
add_subdirectory(${PLOT_DIR}/src ${PLOT_DIR}/build)

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main plot Eigen3::Eigen)
