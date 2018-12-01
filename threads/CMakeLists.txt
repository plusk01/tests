cmake_minimum_required(VERSION 2.8.4)
project(thread_examples)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main pthread)