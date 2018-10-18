cmake_minimum_required(VERSION 2.8.4)
project(simd_playground)

set(CMAKE_BUILD_TYPE Release)

# Create the executables and link them to libs
add_executable(pi src/main.c)
target_link_libraries(pi)
