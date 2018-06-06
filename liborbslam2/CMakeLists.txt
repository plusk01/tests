cmake_minimum_required(VERSION 2.8.4)
project(os2_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})

# Setup cmake paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#################################
#  Download Pangolin Project   #
#################################

set(PANGOLIN_DIR ${CMAKE_CURRENT_BINARY_DIR}/pangolin-download)

find_package(Pangolin QUIET)
if(NOT Pangolin_FOUND)

    # Download pangolin
    configure_file(
      "${CMAKE_MODULE_PATH}/pangolin.cmake.in"
      "${PANGOLIN_DIR}/CMakeLists.txt"
      IMMEDIATE @ONLY)

    execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${PANGOLIN_DIR} )
    execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${PANGOLIN_DIR})

    # add the pangolin CMake module to our current project
    add_subdirectory(${PANGOLIN_DIR}/src ${PANGOLIN_DIR}/build)

    find_package(Pangolin REQUIRED)
else()
    # add the pangolin CMake module to our current project
    add_subdirectory(${PANGOLIN_DIR}/src ${PANGOLIN_DIR}/build EXCLUDE_FROM_ALL)
endif()

#################################

#################################
#  Download ORB_SLAM2 Project   #
#################################

set(OS2_DIR ${CMAKE_CURRENT_BINARY_DIR}/orbslam2-download)

# Download pybind11
configure_file(
  "${CMAKE_MODULE_PATH}/orbslam2.cmake.in"
  "${OS2_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${OS2_DIR} )
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${OS2_DIR})

# add the ORB_SLAM2 CMake module to our current project
add_subdirectory(${OS2_DIR}/src ${OS2_DIR}/build EXCLUDE_FROM_ALL)

execute_process(COMMAND bash "-c" "./build.sh" WORKING_DIRECTORY ${OS2_DIR}/src)

#################################

# Include header files
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories("${OS2_DIR}/src/include" "${OS2_DIR}/src")

include_directories(${Pangolin_INCLUDE_DIRS})

# Create the executables and link them to libs
add_executable(main src/main.cpp)

target_link_libraries(main
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    ${OS2_DIR}/src/Thirdparty/DBoW2/lib/libDBoW2.so
    ${OS2_DIR}/src/Thirdparty/g2o/lib/libg2o.so
    ${OS2_DIR}/src/lib/libORB_SLAM2.so)