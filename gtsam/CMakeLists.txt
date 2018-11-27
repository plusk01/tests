cmake_minimum_required(VERSION 2.8.4)
project(gtsamtutorial)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(GTSAM)

# Include header files
include_directories( ${GTSAM_INCLUDE_DIRS} )

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${GTSAM_LIBRARIES})