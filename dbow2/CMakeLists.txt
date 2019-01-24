cmake_minimum_required(VERSION 3.2)
project(dbow2_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download DBoW2
#

set(DBOW2_DIR ${CMAKE_CURRENT_BINARY_DIR}/dbow2-download)

set(BUILD_Demo OFF CACHE BOOL "Don't build DBoW2 demo")

# Download DBoW2
configure_file(
  "${CMAKE_MODULE_PATH}/dbow2.cmake.in"
  "${DBOW2_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${DBOW2_DIR} )
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${DBOW2_DIR})

# add the DBoW2 CMake module to our current project
add_subdirectory(${DBOW2_DIR}/src ${DBOW2_DIR}/build EXCLUDE_FROM_ALL)

###############################################################################

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})

# Include header files
include_directories(${OpenCV_INCLUDE_DIRS} ${DBOW2_DIR}/src/include)

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main DBoW2 ${OpenCV_LIBS})

# # Create the executables and link them to libs
# add_executable(async src/async.cpp)
# target_link_libraries(async ${CPR_LIBRARIES} Threads::Threads)

# # Create the executables and link them to libs
# add_executable(zipfile src/zipfile.cpp)
# target_link_libraries(zipfile ${CPR_LIBRARIES} Threads::Threads)
