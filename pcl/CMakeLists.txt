cmake_minimum_required(VERSION 3.2)
project(pcl_examples)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)

find_package(PCL 1.9 REQUIRED)
message(STATUS "Found PCL ${PCL_VERSION}")

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# Create the executables and link them to libs
add_executable(pcd_write_test src/pcd_write_test.cpp)
target_include_directories(pcd_write_test PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})

add_executable(registration_pipeline src/registration_pipeline.cpp)
target_include_directories(registration_pipeline PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(registration_pipeline ${PCL_LIBRARIES})
