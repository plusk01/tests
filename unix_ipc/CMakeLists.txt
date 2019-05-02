cmake_minimum_required(VERSION 2.8.4)
project(unixipc)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Create the executables and link them to libs
add_executable(mmap src/mmap.cpp)
target_link_libraries(mmap)