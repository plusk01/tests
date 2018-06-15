cmake_minimum_required(VERSION 2.8.4)
project(udacity_cuda)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -ftree-vectorize ")

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

# CUDA
find_package(CUDA REQUIRED)

# Pass options to NVCC
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3") #-gencode arch=compute_30,code=sm_30

# Include header files
include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )

# Create the executables and link them to libs
cuda_add_executable(lesson2_18 src/lesson2_18.cu)
target_link_libraries(lesson2_18)