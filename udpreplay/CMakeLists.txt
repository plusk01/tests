cmake_minimum_required(VERSION 3.2)
project(udpreplay)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

# Include header files
# include_directories(include)

# Create the executables and link them to libs
add_executable(client src/client.c)
target_link_libraries(client)

# Create the executables and link them to libs
add_executable(server src/server.c)
target_link_libraries(server)
