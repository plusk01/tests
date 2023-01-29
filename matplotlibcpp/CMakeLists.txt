cmake_minimum_required(VERSION 3.15)
project(matplotlibcpp_test)



include(FetchContent)

FetchContent_Declare(matplotlibcpp
  GIT_REPOSITORY https://github.com/lava/matplotlib-cpp.git
  GIT_TAG ef0383f1315d32e0156335e10b82e90b334f6d9f
)
FetchContent_MakeAvailable(matplotlibcpp)


add_executable(main src/main.cpp)
target_link_libraries(main matplotlib_cpp)