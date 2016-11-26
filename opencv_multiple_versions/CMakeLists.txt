cmake_minimum_required(VERSION 2.8.3)

#set(OpenCV_DIR /home/plusk01/libs/opencv2.4/release/share/OpenCV)

# Turn off so linker can link
# See: https://github.com/opencv/opencv/issues/6542
set(CUDA_USE_STATIC_CUDA_RUNTIME 	OFF)

project( VersionTest )


find_package( OpenCV 2.4 REQUIRED PATHS /home/plusk01/libs/opencv2.4/release/share/OpenCV)
#find_package(OpenCV 2.4.13 REQUIRED)
#find_package(OpenCV REQUIRED)

message( "   " )
message( "-- " ${OpenCV_VERSION} )
message( "-- " ${OpenCV_INCLUDE_DIRS} )
message( "-- " ${OpenCV_INSTALL_PATH} )
message( "-- " ${OpenCV_CONFIG_PATH} )
#message( "-- " ${OpenCV_LIBRARY_DIR} )
#message( "Advanced" )
#message( "-- " ${OpenCV_CONFIG_PATH} )
#message( "-- " ${OpenCV_INSTALL_PATH} )
#message( "-- " ${OpenCV_LIB_COMPONENTS} )
#message( "-- " ${OpenCV_MODULES_SUFFIX} )
#message( "-- " ${OpenCV_LIBS} )
message( "   " )

include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories( /home/plusk01/libs/opencv2.4/release/lib  )
add_executable( vertest main.cpp )
target_link_libraries( vertest ${OpenCV_LIBS} )
