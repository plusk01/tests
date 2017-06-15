cmake_minimum_required(VERSION 3.1.0)
project(clib_globals)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)
set(CMAKE_BUILD_TYPE Debug)

# Include header files
include_directories( include )

# add C lib
add_library(mylib src/test.c)

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main mylib)