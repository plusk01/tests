Python to C++ Using `ctypes`
============================

This example uses `ctypes` to pass numpy arrays to a C function for input and output.

See [this](http://stackoverflow.com/questions/5862915/passing-numpy-arrays-to-a-c-function-for-input-and-output) and [this](http://stackoverflow.com/questions/14341549/passing-a-set-of-numpy-arrays-into-c-function-for-input-and-output) SO question.

To compile a `C` file to a shared library:
```C
gcc -fPIC -shared -o mult_by_two.so mult_by_two.c
```

For a good Python/C++ SO question, see [here](http://stackoverflow.com/questions/145270/calling-c-c-from-python)

To compule a `C++` file to a shared library:
```C++
g++ -c -fPIC foo.cpp -o foo.o
g++ -shared -Wl,-soname,libfoo.so -o libfoo.so  foo.o
```

**Note**: There is a difference between `g++` and `gcc`, see [here](http://stackoverflow.com/a/172592)

--------------------------------------------------------

- NumPy reference on [ctypeslib](http://docs.scipy.org/doc/numpy/reference/routines.ctypeslib.html)
- [Custom Structures](http://stackoverflow.com/questions/15667361/callback-with-custom-types-in-python-ctypes)