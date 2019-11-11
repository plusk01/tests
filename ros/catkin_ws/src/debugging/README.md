ROS/C++ Debugging
=================

The following provides a quick guide to debugging (particularly `SIGSEGV`) in C++/ROS.

## Non-invasive

You can use use `gdb` to debug a core dump from a segfault. But you must make the following system changes.

```bash
# make sure to allow core dumps (per session)
ulimit -c unlimited

# check ulimits
ulimit -a

# Apport will try and consume core files for crash management.
# Instead, we want the kernel to just store them in /tmp
sudo sysctl -w kernel.core_pattern=/tmp/core-%e.%p.%h.%t

# Note: the core_pattern can be checked with
cat /proc/sys/kernel/core_pattern

# Note: core_pattern can be returned to default by restarting apport
sudo systemctl restart apport
```

Once you have a core dump, do the following

```bash
# Open core dump in gdb
gdb ~/dev/tests/ros/catkin_ws/devel/.private/debugging/lib/debugging/debugging core-debugging.<pid>.<hostname>.<time>

# Note: the following doesn't seem to work:
gdb -c <core>
(gdb) symbol-file /home/plusk01/dev/tests/ros/catkin_ws/devel/.private/debugging/lib/debugging/debugging
(gdb) sharedlibrary
```

## Third Party Extensions

- Use `google::InstallFailureSignalHandler()` from `glog`.

### Resources

- [Julia Evans: Debugging a segfault on Linux](https://jvns.ca/blog/2018/04/28/debugging-a-segfault-on-linux/)
