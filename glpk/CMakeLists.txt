cmake_minimum_required(VERSION 2.8.4)
project(glpk_test)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download GLPK
#

set(GLPK_DIR ${CMAKE_CURRENT_BINARY_DIR}/glpk-download)

# Download GLPK
configure_file(
  "${CMAKE_MODULE_PATH}/glpk.cmake.in"
  "${GLPK_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${GLPK_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${GLPK_DIR})

# For convenience
set(GLPK_INCLUDE_DIRS ${GLPK_DIR}/install/include)
set(GLPK_LIBS -L${GLPK_DIR}/install/lib;libglpk.a)

###############################################################################

# Create the executables and link them to libs
add_executable(main src/main.cpp)
target_include_directories(main PRIVATE ${GLPK_INCLUDE_DIRS})
target_link_libraries(main ${GLPK_LIBS})

