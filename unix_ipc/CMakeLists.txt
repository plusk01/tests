cmake_minimum_required(VERSION 2.8.4)
project(unixipc)

# for some reason snapdragon doesn't like the
# proper CMake way of doing this (it's cmake v2.8.12.2)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

include_directories(include)

# Create the executables and link them to libs
add_executable(mmapdemo src/mmapdemo.cpp)
target_link_libraries(mmapdemo)

add_executable(mmap_server src/mmap_server.cpp)
target_link_libraries(mmap_server pthread)

add_executable(mmap_client src/mmap_client.cpp)
target_link_libraries(mmap_client pthread)

add_executable(shmem_server src/shmem_server.cpp)
target_link_libraries(shmem_server pthread)

add_executable(shmem_client src/shmem_client.cpp)
target_link_libraries(shmem_client pthread)