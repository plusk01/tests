cmake_minimum_required(VERSION 2.8.3)
project(camera_model)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
# include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV 3 REQUIRED)
message(STATUS "OpenCV Version: ${OpenCV_VERSION} (${OpenCV_DIR})")
# include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
message(STATUS "Eigen Version: ${EIGEN3_VERSION_STRING} (${EIGEN3_INCLUDE_DIRS})")
# include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIRS})

# include_directories(include)

#
# camera_model library
#
add_library(${PROJECT_NAME}
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)
target_include_directories(${PROJECT_NAME}
    PRIVATE ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
    PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

#
# intrinsic calibration binary
#
add_executable(calibration src/intrinsic_calib.cc)
target_include_directories(calibration
    PRIVATE ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
            ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}
            ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(calibration ${PROJECT_NAME} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
