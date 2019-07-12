#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
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

  std::unique_ptr<Client> client;

  //
  // Decide which comms to use
  //

  if (argc != 2) {
    usage(argv);
    return -1;
  }

  if (strcmp(argv[1], "mmap") == 0) {
    client.reset(new MmapClient("data.bin"));
  } else if (strcmp(argv[1], "shmem") == 0) {
    client.reset(new ShmemClient);
  } else if (strcmp(argv[1], "udp") == 0) {

  } else {
    std::cout << "Communications type '" << argv[1] << "' not understood." << std::endl;
    usage(argv);
    return -1;
  }

  // keep track of how many messages were recvd
  uint32_t count = 0;
  uint32_t id = 0;
  uint32_t missed = 0;

  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::min();
  double avg = 0;

  bool first = true;

  while (!stop) {

    // time we started waiting at
    auto start = std::chrono::steady_clock::now();

    msg_t msg; msg.id = 1;
    bool recvd = client->read(&msg);

    // check for a timeout
    if (!recvd) break;

    // what was the first message id we saw?
    if (first) {
      first = false;
      id = msg.id;
      std::cout << "Got first message: " << id << std::endl;
    }

    // make sure the message ids are synced
    if (msg.id != id) {
      missed += std::abs(static_cast<int32_t>(msg.id - id));
      id = msg.id;
    }

    // calculate transmission latency
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    auto elapsed = diff(msg.t, t);

    double usec = elapsed.tv_sec/1e-6 + elapsed.tv_nsec*1e-3;
    // std::cout << "count (id): " << count << " (" << id << ") [" << usec << " usec]" << std::endl;

    // time stats
    if (usec > max) max = usec;
    if (usec < min) min = usec;
    avg = 1.0/(count+1) * (count*avg + usec);

    if (usec > 3000) {
      std::cout << "usec: " << usec << std::endl;
    }

    id++;
    count++;
  }

  std::cout << "received " << count-missed << " messages." << std::endl;
  std::cout << "missed " << missed << " messages." << std::endl;
  std::cout << "mean: " << avg << " us.  max: " << max << " us.  min: " << min << " us." << std::endl;

  return 0;
}
