cmake_minimum_required(VERSION 2.8.9)
project(csdp C)

# Project paths
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#
# Download csdp
#

set(CSDP_DIR ${CMAKE_CURRENT_BINARY_DIR}/csdp-download)
set(CSDP_SRCDIR ${CSDP_DIR}/src)

# Download csdp
configure_file(
  "${CMAKE_MODULE_PATH}/csdp-src.cmake.in"
  "${CSDP_DIR}/CMakeLists.txt"
  IMMEDIATE @ONLY)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" . WORKING_DIRECTORY ${CSDP_DIR})
execute_process(COMMAND ${CMAKE_COMMAND} --build . WORKING_DIRECTORY ${CSDP_DIR})

# =============================================================================

find_package(LAPACK REQUIRED)

file(GLOB csdp_srcs "${CSDP_SRCDIR}/lib/*.c")
add_library(csdp STATIC ${csdp_srcs})
target_include_directories(csdp PUBLIC ${CSDP_SRCDIR}/include)
target_link_libraries(csdp PUBLIC ${LAPACK_LIBRARIES} m)
# install(TARGETS libcsdp
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib
#         RUNTIME DESTINATION bin)

# add_executable(csdp solver/csdp.c)
# target_link_libraries(csdp libcsdp)
# install(TARGETS csdp DESTINATION bin)

# add_executable(theta theta/theta.c)
# target_link_libraries(theta libcsdp)
# install(TARGETS theta DESTINATION bin)

# add_executable(graphtoprob theta/graphtoprob.c)
# target_link_libraries(graphtoprob libcsdp)
# install(TARGETS graphtoprob DESTINATION bin)

# add_executable(complement theta/complement.c)
# target_link_libraries(complement libcsdp)
# install(TARGETS complement DESTINATION bin)

# add_executable(rand_graph theta/rand_graph.c)
# target_link_libraries(rand_graph libcsdp)
# install(TARGETS rand_graph DESTINATION bin)

# enable_testing()
# add_test (NAME theta1 COMMAND csdp ${PROJECT_SOURCE_DIR}/test/theta1.dat-s)
# add_test (NAME g50 COMMAND theta ${PROJECT_SOURCE_DIR}/test/g50)