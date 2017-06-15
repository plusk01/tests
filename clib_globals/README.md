Multiple instances of C library with C++ wrapper
================================================

Investigation of using multiple instances of a shared C library that has global variables in it. I use `dlopen` and `dlsym` to dynamically load the C library.

The behaviour of `dlopen` will create a new 'instance' of the shared library -- it uses reference counting and will just return a pointer to the previously loaded library.

To get around this, I created a `libmylib0.so` and `libmylib1.so`. They are exactly the same, just different names. The `CLibWrapper` class has a static `socounter` that is incremented each time an `so` is loaded. Therefore, in order for this heinous hack to work, you need to know a priori how many `so`'s you will load and you need to create them.

I suppose you could have the `ctor` create a copy with a different name and put it in `/tmp`, but again -- this is terrible code.

## References ##

- [c++ dlopen mini HOWTO](http://tldp.org/HOWTO/C++-dlopen/thesolution.html)
- [c usage of dlopen](https://www.dwheeler.com/program-library/Program-Library-HOWTO/x172.html)
- [behaviour of dlopen and multiple 'instances'](https://stackoverflow.com/a/42472148/2392520)
