cmake_minimum_required(VERSION 3.2)
project(http_image)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download zipper
#

set(ZIPPER_DIR ${CMAKE_CURRENT_BINARY_DIR}/zipper-download)

# Download zipper
configure_file(
  "${CMAKE_MODULE_PATH}/zipper.cmake.in"
  "${ZIPPER_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${ZIPPER_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${ZIPPER_DIR})

# add the zipper CMake module to our current project
add_subdirectory(${ZIPPER_DIR}/src ${ZIPPER_DIR}/build EXCLUDE_FROM_ALL)

set(ZIPPER_HEADERS ${ZIPPER_DIR}/src)
set(ZIPPER_LIBRARIES Zipper-static)

###############################################################################

# Include header files
message("Zipper: ${ZIPPER_HEADERS} ${ZIPPER_LIBRARIES}")
include_directories(${ZIPPER_HEADERS})


# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${ZIPPER_LIBRARIES})