#pragma once

#include <iostream>
#include <dlfcn.h>

// // Hide the C code definitions so that we don't polute the global namespace
// namespace mylib {
// #include <test.h>
// }

class CLibWrapper
{
	public:
		CLibWrapper() {
			// This is ugly, not scalable, and just pure evil...
			std::string filename = "./libmylib" + std::to_string(socounter++) + ".so";
			void* handle = dlopen(filename.c_str(), RTLD_NOW | RTLD_LOCAL | RTLD_DEEPBIND);

			// Check for errors...
			if (!handle)
				std::cerr << "Cannot load library: " << dlerror() << std::endl;

			// reset errors (is this necessary here?)
			dlerror();

			// load the symbols (aka, functions)
			// Have to cast from a void* to an int(*)()
			// https://stackoverflow.com/a/33064943/2392520
			c_read_global = (int(*)(void)) dlsym(handle, "read_global");
			c_inc_global = (void(*)(void)) dlsym(handle, "inc_global");
		}
		
		int read_global() {
			return c_read_global();
		}

		void inc_global() {
			c_inc_global();
		}

		int read_static() {

		}

		void inc_static() {
			
		}


	private:
		int (*c_read_global)(void);
		void (*c_inc_global)(void);

		static int socounter;
};

int CLibWrapper::socounter = 0;