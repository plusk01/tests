cmake_minimum_required(VERSION 2.8.4)
project(eigen_playground)

set (CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

option(ENABLE_MKL "Use MKL with Eigen" OFF)
if (ENABLE_MKL)
    find_package(MKL)
    if(MKL_FOUND)
        message(STATUS "MKL found at: ${MKL_LIBRARIES}")
    else()
        message(STATUS "MKL not found.")
    endif()
endif()

# Find dependent libraries
find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: " ${OpenCV_VERSION})
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: " ${EIGEN3_VERSION_STRING})

# Include header files
include_directories( ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )

# Create the executables and link them to libs
add_executable(matpow src/matpow.cpp)
target_link_libraries(matpow)

add_executable(reshape src/reshape.cpp)
target_link_libraries(reshape)

add_executable(norm src/norm.cpp)
target_link_libraries(norm)

add_executable(conservative_resize src/conservative_resize.cpp)
target_link_libraries(conservative_resize)

add_executable(perspective_transform src/perspective_transform.cpp)
target_link_libraries(perspective_transform)

add_executable(idx_lookup src/idx_lookup.cpp)
target_link_libraries(idx_lookup)

add_executable(init src/init.cpp)
target_link_libraries(init)

add_executable(eigensolver src/eigensolver.cpp)
target_link_libraries(eigensolver)

add_executable(fnargs src/fnargs.cpp)
target_link_libraries(fnargs)

add_executable(array src/array.cpp)
target_link_libraries(array)

add_executable(geometry src/geometry.cpp)
target_link_libraries(geometry ${OpenCV_LIBS})

add_executable(reflections src/reflections.cpp)
target_link_libraries(reflections)

add_executable(graphutils src/graphutils.cpp)
target_link_libraries(graphutils)

add_executable(mapinout src/mapinout.cpp)
target_link_libraries(mapinout)

add_executable(arun src/arun.cpp)
target_link_libraries(arun ${OpenCV_LIBS})

add_executable(sparse src/sparse.cpp)
target_link_libraries(sparse)

add_executable(visitors src/visitors.cpp)
target_link_libraries(visitors)

add_executable(euler src/euler.cpp)
target_link_libraries(euler)

add_executable(mkl src/mkl.cpp)
target_link_libraries(mkl)
if(MKL_FOUND)
    target_include_directories(mkl PRIVATE ${MKL_INCLUDE_DIR})
    target_compile_definitions(mkl PRIVATE -DEIGEN_USE_MKL_ALL)
    target_link_libraries(mkl PRIVATE ${MKL_LIBRARIES})
endif()
