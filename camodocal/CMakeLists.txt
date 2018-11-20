cmake_minimum_required(VERSION 2.8.4)
project(camodocal_examples)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

add_subdirectory(lib/camera_model)

# Include header files
include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(distortion src/distortion.cpp)
target_link_libraries(distortion ${OpenCV_LIBS} camera_model)
