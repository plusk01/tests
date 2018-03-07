cmake_minimum_required(VERSION 2.8.4)
project(cxx_const_ptrs)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Include header files
include_directories(include)

# Create the executables and link them to libs
add_executable(main src/main.cpp)