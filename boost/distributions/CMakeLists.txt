cmake_minimum_required(VERSION 3.2)
project(distributions)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(Boost REQUIRED)

# # Include header files
# include_directories(${Boost_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(ncx2 src/ncx2.cpp)
target_link_libraries(ncx2 PUBLIC Boost::boost)
