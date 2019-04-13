cmake_minimum_required(VERSION 3.2)
project(asio_udp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED COMPONENTS system thread)

# Include header files
include_directories(${Boost_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${Boost_LIBRARIES})
