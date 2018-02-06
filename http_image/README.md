HTTP Image Request
==================

This example demonstrates using C++ and the [cpr](https://github.com/whoshuu/cpr) project (a wrapper for `libcurl`) to perform a `GET` request to download an image.

Note that the code in this example is not ideal for large files because the data has to reside in memory until after it is written to file. See [this issue](https://github.com/whoshuu/cpr/issues/86) for more discussion.
