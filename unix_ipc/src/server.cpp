#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <limits>
#include <memory>

// ctrl+c handler
#include <cstdlib>
#include <csignal>

#include "utils.h"
#include "abstract.h"
#include "mmap.h"
#include "shmem.h"
#include "msg.h"

// interrupt signal (ctrl+c) handler
volatile sig_atomic_t stop = 0;
void handle_sigint(int s) { stop = 1; }

int main(int argc, char const *argv[])
{
  // register sigint handler
  signal(SIGINT, handle_sigint);

  std::unique_ptr<Server> server;

  //
  // Decide which comms to use
  //

  if (argc != 2) {
    usage(argv);
    return -1;
  }

  if (strcmp(argv[1], "mmap") == 0) {
    server.reset(new MmapServer("data.bin"));
  } else if (strcmp(argv[1], "shmem") == 0) {
    server.reset(new ShmemServer);
  } else if (strcmp(argv[1], "udp") == 0) {

  } else {
    std::cout << "Communications type '" << argv[1] << "' not understood." << std::endl;
    usage(argv);
    return -1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::min();
  double avg = 0;

  constexpr int MSG_COUNT = 1000;

  // for (int i=0; i<MSG_COUNT; ++i) {
  uint32_t i = 0;
  while (!stop) {
    msg_t msg;
    msg.id = i;
    for (int j=0; j<NUM_PWM; ++j) {
      msg.pwm[j] =  i + j*0.1f + 0.01f;
    }
    clock_gettime(CLOCK_MONOTONIC, &msg.t);

    server->send(&msg);

    // calculate time to send
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    auto elapsed = diff(msg.t, t);
    double usec = elapsed.tv_sec/1e-6 + elapsed.tv_nsec*1e-3;

    // time stats
    if (usec > max) max = usec;
    if (usec < min) min = usec;
    avg = 1.0/(i+1) * (i*avg + usec);

    i++; // inc count
    std::cout << "sent " << msg.id << " in " << usec << " usec" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  std::cout << "done" << std::endl;

  std::cout << "sent " << i << " messages." << std::endl;
  std::cout << "mean: " << avg << " us.  max: " << max << " us.  min: " << min << " us." << std::endl;

  return 0;
}
