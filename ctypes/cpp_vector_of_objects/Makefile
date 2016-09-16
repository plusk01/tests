all: lib.so

lib.so: model_python.cpp
	g++ -Wall -c -fPIC model_python.cpp -o model_python.o
	g++ -Wall -shared -Wl,-soname,lib.so -o lib.so model_python.o -lopencv_core
