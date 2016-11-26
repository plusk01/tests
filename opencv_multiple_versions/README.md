OpenCV Multiple Versions
========================

Using multiple OpenCV versions on one machine for different projects.

### CMake Installation ###

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

### Resources ###

- [Side by Side](http://code.litomisky.com/2014/03/09/how-to-have-multiple-versions-of-the-same-library-side-by-side/)
- [Python, virtualenv](http://stackoverflow.com/questions/9592389/is-it-possible-to-run-opencv-python-binding-from-a-virtualenv)
