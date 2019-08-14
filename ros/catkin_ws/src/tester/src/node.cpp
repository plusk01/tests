#include <iostream>

#include <ros/ros.h>

#include <libtest/core.h>

int main(int argc, char const *argv[])
{
  int ret = libtest::core();
  std::cout << "got " << ret << std::endl;
  return 0;
}