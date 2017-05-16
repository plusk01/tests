all: lib.so

lib.so: PyEigen.cpp
	g++ -Wall -c -fPIC PyEigen.cpp `pkg-config --cflags eigen3` -o PyEigen.o
	g++ -Wall -shared -Wl,-soname,lib.so -o lib.so PyEigen.o
