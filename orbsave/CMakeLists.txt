cmake_minimum_required(VERSION 3.2)
project(orbsave)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})

# Include header files
include_directories(${OpenCV_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(save src/save.cpp)
target_link_libraries(save ${OpenCV_LIBS})

add_executable(load src/load.cpp)
target_link_libraries(load ${OpenCV_LIBS})
