all: lib.so

lib.so: cvmat_python.cpp
	g++ -Wall -c -fPIC cvmat_python.cpp -o cvmat_python.o
	g++ -Wall -shared -Wl,-soname,lib.so -o lib.so cvmat_python.o -lopencv_core
