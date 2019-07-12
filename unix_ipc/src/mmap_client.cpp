#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <exception>
#include <limits>

#include "msg.h"

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

// ctrl+c handler
#include <cstdlib>
#include <csignal>

// interrupt signal (ctrl+c) handler
volatile sig_atomic_t stop = 0;
void handle_sigint(int s) { stop = 1; }

class Client
{
public:
  Client(const std::string path)
  : path_(path)
  {
    if (!init()) {
      std::throw_with_nested(std::runtime_error("could not initialize"));
    }
  }
  ~Client() { deinit(); }

  bool read(msg_t * msg)
  {
    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 3;

    // lock -- predicate (the mutex / condvar) -- unlock pattern
    // TODO: catch spurious wakeups?
    pthread_mutex_lock(&msg_ptr_->mutex);
    int ret = pthread_cond_timedwait(&msg_ptr_->condvar, &msg_ptr_->mutex, &timeout);

    // we timed out!
    if (ret != 0) {
      pthread_mutex_unlock(&msg_ptr_->mutex);
      return false;
    }

    // copy data locally
    std::memcpy(&msg->id, &msg_ptr_->id, sizeof(msg->id));
    std::memcpy(&msg->pwm, &msg_ptr_->pwm, sizeof(msg->pwm));
    std::memcpy(&msg->t, &msg_ptr_->t, sizeof(msg->t));

    pthread_mutex_unlock(&msg_ptr_->mutex);
    return true;
  }

private:
  const std::string path_;
  msg_t * msg_ptr_ = nullptr;

  bool init()
  {
    // Open the file with R/W permissions, create if it does not exist
    const int fd = open(path_.c_str(), O_RDWR | O_CREAT, 0600);
    if (fd == -1) return false;

    // Stretch the file size to the desired size
    if (posix_fallocate(fd, 0, sizeof(msg_t)) != 0) return false;

    // Memory map the file with R/W permissions
    void * ptr = mmap(0, sizeof(msg_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

    close(fd);
    return true;
  }

  // --------------------------------------------------------------------------

  void deinit()
  {
    munmap(reinterpret_cast<void *>(msg_ptr_), sizeof(msg_t));
  }
    
};

// ============================================================================
// ============================================================================

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

// ============================================================================
// ============================================================================

int main(int argc, char const *argv[])
{
  // register sigint handler
  signal(SIGINT, handle_sigint);

  Client client("data.bin");

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
    bool recvd = client.read(&msg);

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