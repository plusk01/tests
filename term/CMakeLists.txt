cmake_minimum_required(VERSION 2.8.4)
project(term)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(Curses REQUIRED)

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_link_libraries(main ${CURSES_LIBRARIES})
target_include_directories(main PUBLIC ${CURSES_INCLUDE_DIR})

add_executable(mytest src/test.cpp)
target_link_libraries(mytest ${CURSES_LIBRARIES})
target_include_directories(mytest PUBLIC ${CURSES_INCLUDE_DIR})

add_executable(colors src/colors.cpp)
target_link_libraries(colors ${CURSES_LIBRARIES})
target_include_directories(colors PUBLIC ${CURSES_INCLUDE_DIR})
