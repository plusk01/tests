#include <iostream>

#include "test.h"

int main(int argc, char const *argv[])
{
	std::cout << "Global: " << read_global() << std::endl;
	inc_global();
	std::cout << "Global: " << read_global() << std::endl;

	std::cout << "Static: " << read_static() << std::endl;
	inc_static();
	std::cout << "Static: " << read_static() << std::endl;


	return 0;
}