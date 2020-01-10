cmake_minimum_required(VERSION 2.8.4)
project(dlib_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download dlib
#

set(DLIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/dlib-download)

# Download dlib
configure_file(
  "${CMAKE_MODULE_PATH}/dlib.cmake.in"
  "${DLIB_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${DLIB_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${DLIB_DIR})

add_subdirectory(${DLIB_DIR}/src ${DLIB_DIR}/build)
###############################################################################

# Create the executables and link them to libs
add_executable(max_cost_assignment_ex src/max_cost_assignment_ex.cpp)
target_link_libraries(max_cost_assignment_ex dlib::dlib)
