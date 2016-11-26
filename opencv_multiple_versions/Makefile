CPP = g++

# OpenCV 2.4
CPPFLAGS=-I/home/plusk01/libs/opencv2.4/release/include
LDFLAGS=-L/home/plusk01/libs/opencv2.4/release/libs

# OpenCV something? (3.1.0; from where?)
#CPPFLAGS=`pkg-config --cflags opencv`
#LDFLAGS=`pkg-config --libs-only-L opencv`

# ROS Kinetic OpenCV 3.1.0-dev
#CPPFLAGS=`pkg-config --cflags opencv-3.1.0-dev`
#LDFLAGS=`pkg-config --libs-only-L opencv-3.1.0-dev`


all: vertest

vertest: main.cpp
	$(CPP) $(CPPFLAGS) $(LDFLAGS) $^ -o $@

clean:
	rm vertest
