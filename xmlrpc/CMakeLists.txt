cmake_minimum_required(VERSION 3.10)
project(xmlrpc_examples VERSION 0.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

include(cmake/fastrpc.cmake)

# Create the executables and link them to libs
add_executable(client src/client.cpp)
target_link_libraries(client fastrpc)


