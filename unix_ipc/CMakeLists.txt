cmake_minimum_required(VERSION 2.8.4)
project(unixipc)

# for some reason snapdragon doesn't like the
# proper CMake way of doing this (it's cmake v2.8.12.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

include_directories(include)

# Create the executables and link them to libs
add_executable(server src/server.cpp src/mmap_impl.cpp src/shmem_impl.cpp src/utils.cpp)
target_link_libraries(server pthread)

add_executable(client src/client.cpp src/mmap_impl.cpp src/shmem_impl.cpp src/utils.cpp)
target_link_libraries(client pthread)