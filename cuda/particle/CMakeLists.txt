cmake_minimum_required(VERSION 3.2)
project(particle LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# CUDA
find_package(CUDA REQUIRED)

# Pass options to NVCC
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -O3") #-gencode arch=compute_30,code=sm_30

# Include header files
include_directories(include ${CUDA_INCLUDE_DIRS})

add_library(particles STATIC src/randomize.cpp src/particle.cu src/v3.cu)
set_target_properties(particles PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# set_source_files_properties( src/main.cpp src/particle.cpp src/v3.cpp PROPERTIES CUDA_SOURCE_PROPERTY_FORMAT OBJ )

# # Create the executables and link them to libs
# cuda_add_executable(particle src/main.cpp src/particle.cpp src/v3.cpp)
# # target_link_libraries(particle ${CUDA_LIBRARIES})