#include <iostream>

#include "utils.h"

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec-start.tv_nsec)<0) {
    // rollover in seconds
    temp.tv_sec = end.tv_sec-start.tv_sec-1;
    temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec-start.tv_sec;
    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
  }
  return temp;
}

// ----------------------------------------------------------------------------

void usage(char const *argv[])
{
  std::cout << std::endl;
  std::cout << "usage:" << std::endl;
  std::cout << "\t" << argv[0] << " mmap|shmem|udp" << std::endl;
  std::cout << std::endl;
}
