cmake_minimum_required(VERSION 3.1)
project(TestVisualizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# If you haven't installed Open3D system-wide, you will need to set the
# CMAKE_PREFIX_PATH=<install_dir>, e.g.,
# cmake -DCMAKE_PREFIX_PATH=/home/plusk01/dev/o3d/Open3D/build/install ..
find_package(Open3D)

# Open3D
if (Open3D_FOUND)
    message(STATUS "Found Open3D ${Open3D_VERSION} at ${Open3D_CONFIG}")
    add_executable(TestVisualizer src/TestVisualizer.cpp)
    target_link_libraries(TestVisualizer Open3D::Open3D)
else()
    message(SEND_ERROR "Open3D not found")
endif()
