pybind11 Example Project
========================

Uses the [pybind11](https://github.com/pybind/pybind11) project to expose C++11 functionality into a Python (2.7 or 3.x) interface.

You can either build the Python dynamic module (`pybind_example.so`) with CMake and then ensure that you are in the same directory as the `.so` when you want to use in Python, like so:

```bash
$ mkdir build && cd build
$ cmake ..
$ make
$ python
```

In the python interpreter:

```python
>>> import pybind_example
>>> pybind_example.add(5,5)
10
```

Which works, but is obviously clunky. Alternatively, you would use Python's `setuptools` to install the `.so` into your `site-packages` directory so and Python code from anywhere can use the `.so` module. From the root project directory (where `setup.py` is), simply do:

```bash
$ pip install . --user
```

This runs `cmake` and `make` in the background, creates a Python egg for distribution, and then copies it to your site-packages in your home direcory. Now you can `import pybind_example` from anywhere on your computer, provided you have access to an interpreter with the appropriate version as the egg was built for. Note that you can check all your Python sites with the command: `python -m site`.
