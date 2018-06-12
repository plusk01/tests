cmake_minimum_required(VERSION 2.8.4)
project(openmp_playground)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -ftree-vectorize ")

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

# Include header files
include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )

# Create the executables and link them to libs
add_executable(parfor src/parfor.cpp)
target_link_libraries(parfor)

# Create the executables and link them to libs
add_executable(mandelbrot src/mandelbrot.cpp)
target_link_libraries(mandelbrot)

# Create the executables and link them to libs
add_executable(playground src/playground.cpp)
target_link_libraries(playground)