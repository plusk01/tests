cmake_minimum_required(VERSION 2.8.4)
project(normalized_image_plane)

set(CMAKE_CXX_STANDARD 11)

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})

# Include header files
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${OpenCV_LIBS})
