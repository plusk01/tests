#include <iostream>

#include "clibwrapper.hpp"

int main(int argc, char const *argv[])
{
	// Instantiate a C++ object that uses a C shared library with globals
	CLibWrapper obj1;

	std::cout << "Obj1 Global: " << obj1.read_global() << std::endl;
	obj1.inc_global();
	std::cout << "Obj1 Global: " << obj1.read_global() << std::endl;

	// std::cout << "Static: " << read_static() << std::endl;
	// inc_static();
	// std::cout << "Static: " << read_static() << std::endl;

	// ------------------------------------------------------------------------
	// Instantiate another object that is using the same shared library

	CLibWrapper obj2;

	std::cout << "Obj2 Global: " << obj2.read_global() << std::endl;
	obj2.inc_global();
	std::cout << "Obj2 Global: " << obj2.read_global() << std::endl;


	return 0;
}