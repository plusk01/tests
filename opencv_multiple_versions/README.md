OpenCV Multiple Versions
========================

Using multiple OpenCV versions on one machine for different projects.

## Installing OpenCV From Source ##

Clone the repo and install which ever branch / version you want. Use `cmake` and make sure to set the prefix path:
```bash
cmake -DCMAKE_INSTALL_PREFIX=/home/plusk01/libs/opencv2.4/release \
	-DCMAKE_BUILD_TYPE=RELEASE ~/dev/opencv/ 
```

Also, you can use `ccmake` to bring up a `curses` terminal gui thing to select all the options. If you want Python, make sure Python is ON.

## CMake Installation of this project ##

This took a lot of work, and it's still a little confusing to me. It's important to know about the environment variable `$CMAKE_PREFIX_PATH` which, whith ROS Kinetic installed, defaults to: `/opt/ros/kinetic`. This will force `find_package(OpenCV REQUIRED)` to find OpenCV-3.1.0-dev, which is included with ROS Kintetic. Change this to the location where your other OpenCV installation is found in order to use CMake with that version of OpenCV:
```bash
$ export CMAKE_PREFIX_PATH=/home/plusk01/libs/opencv2.4/release
```

It's also worth noting that with CUDA 7.5 installed on my machine and having compiled OpenCV 2.4.13 with CUDA support, I get a linker error after running `cmake ..` and running `make`:
```bash
/usr/bin/ld: cannot find -lopencv_dep_cudart
collect2: error: ld returned 1 exit status
```

To get past it, I added `set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)` to my `CMakeLists.txt` file. However, I still get an error the first time I run `cmake .. && make`. Therefore, to correctly install my compiled OpenCV I do the following:
```bash
$ mkdir build && cd build
$ cmake ..
$ make
# the linker will fail to link -lopencv_dep_cudart
$ cmake ..
$ make
# success
```

See `CMakeLists.txt` for more info. Alternatively, just use the `Makefile` provided.

## Python Setup ##

I need to repent of not having used virtualenvs ever since I've been doing more scientific Python computing. The best way to use side-by-side OpenCV versions in Python is... virtualenvs!

Use the `virtualenvwrapper` and create a virtualenv. Mine is called `cv2.4.13` because I'm always confused about OpenCV versioning ("Is this cv3 version 3? or...").

Wherever you set the `-DCMAKE_INSTALL_PREFIX` is where you'll find the python installation (`cv2.so` and `cv.py`). Create a symlink from your virtualenv's `site-packages` to this files:
```bash
$ ln -s /home/plusk01/libs/opencv2.4/release/lib/python2.7/dist-packages/cv2.so /home/plusk01/.virtualenvs/cv2.4.13/lib/python2.7/site-packages/
$ ln -s /home/plusk01/libs/opencv2.4/release/lib/python2.7/dist-packages/cv.py /home/plusk01/.virtualenvs/cv2.4.13/lib/python2.7/site-packages/
```

**Note**: I'm not sure how to best install ROS in a virtualenv... Perhaps I'll look into that later. For the time being, in order to have a somewhat clean virtualenv, I clear the `$PYTHONPATH` env variable (see [here](http://stackoverflow.com/a/15887589/2392520)). It seems that ROS adds it's own dist-packages (`/opt/ros/kinetic/lib/python2.7/dist-packages`) to the `PYTHONPATH` which makes all ROS things show up in a virtualenv. If you `unset PYTHONPATH` those ROS things won't show up (although it still seems that `cv2.so` from ROS is still being found?). What I do is add `unset PYTHONPATH` to the `postactivate` script for my virtualenv (`~/.virtualenvs/cv2.4.13/bin/postactivate`):
```bash
#!/bin/bash
# This hook is sourced after this virtualenv is activated.
unset PYTHONPATH
```

Having done all this, I can see that I'm using my compiled version of OpenCV 2.4.13 by doing the following in an `ipython` shell in my virtualenv:
```python
In [1]: import cv2

In [2]: cv2.__version__
Out[2]: '2.4.13.1'

In [3]: cv2.__file__
Out[3]: '/home/plusk01/.virtualenvs/cv2.4.13/lib/python2.7/site-packages/cv2.so'
```


## Resources ##

- [Side by Side](http://code.litomisky.com/2014/03/09/how-to-have-multiple-versions-of-the-same-library-side-by-side/)
- [Python, virtualenv](http://stackoverflow.com/questions/9592389/is-it-possible-to-run-opencv-python-binding-from-a-virtualenv)
