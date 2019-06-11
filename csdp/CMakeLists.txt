cmake_minimum_required(VERSION 3.2)
project(sdptests)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# add the csdp target
add_subdirectory(lib/csdp)

# Create the executables and link them to libs
add_executable(lp src/lp.cpp)
target_link_libraries(lp csdp)

add_executable(example src/example.c)
target_link_libraries(example csdp)