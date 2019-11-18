cmake_minimum_required(VERSION 2.8.4)
project(qpoases_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download qpOASES
#

set(QPOASES_DIR ${CMAKE_CURRENT_BINARY_DIR}/qpoases-download)

# Download qpOASES
configure_file(
  "${CMAKE_MODULE_PATH}/qpoases.cmake.in"
  "${QPOASES_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${QPOASES_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${QPOASES_DIR})

# For convenience (link to static library)
set(QPOASES_INCLUDE_DIRS ${QPOASES_DIR}/install/include)
set(QPOASES_LIBS -L${QPOASES_DIR}/install/lib;libqpOASES.a)

###############################################################################

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_include_directories(main PRIVATE ${QPOASES_INCLUDE_DIRS})
target_link_libraries(main ${QPOASES_LIBS})

