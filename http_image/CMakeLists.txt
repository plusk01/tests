cmake_minimum_required(VERSION 3.2)
project(http_image)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download CPR
#

set(CPR_DIR ${CMAKE_CURRENT_BINARY_DIR}/cpr-download)

# Don't build CPR tests and try to use the system libcurl (for speed)
set(BUILD_CPR_TESTS OFF CACHE BOOL "Don't build cpr tests")
find_package(CURL)
if(CURL_FOUND)
    set(USE_SYSTEM_CURL ON CACHE BOOL "Use the system curl for faster builds")
endif()

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

find_package(Threads REQUIRED)

# Include header files
include_directories(${CPR_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${CPR_LIBRARIES})

# Create the executables and link them to libs
add_executable(async src/async.cpp)
target_link_libraries(async ${CPR_LIBRARIES} Threads::Threads)
