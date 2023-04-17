cmake_minimum_required(VERSION 3.20)
project(pangolin_test)

set(CMAKE_CXX_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE)
    # Options: Debug, Release, MinSizeRel, RelWithDebInfo
    message(STATUS "No build type selected, default to Release")
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose build type." FORCE)
endif()


find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING} ${EIGEN3_VERSION}")



include(FetchContent)

FetchContent_Declare(pangolin
  GIT_REPOSITORY  https://github.com/stevenlovegrove/Pangolin
  GIT_TAG         v0.6
)
FetchContent_MakeAvailable(pangolin)

message(STATUS "Pangolin: ${Pangolin_LIBRARIES}")


add_executable(main src/main.cpp)
target_link_libraries(main pangolin Eigen3::Eigen)