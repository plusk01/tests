cmake_minimum_required(VERSION 2.8.4)
project(eigen_playground)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

# Include header files
include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )

# Create the executables and link them to libs
add_executable(matpow src/matpow.cpp)
target_link_libraries(matpow ${Eigen3_LIBS})

add_executable(reshape src/reshape.cpp)
target_link_libraries(reshape ${Eigen3_LIBS})

add_executable(norm src/norm.cpp)
target_link_libraries(norm ${Eigen3_LIBS})
