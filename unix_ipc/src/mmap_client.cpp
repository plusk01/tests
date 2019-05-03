#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <exception>

#include "msg.h"

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

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
    // return false;

    struct timespec timeout;
    clock_gettime(CLOCK_REALTIME, &timeout);
    timeout.tv_sec += 3;

    // lock -- predicate (the mutex / condvar) -- unlock pattern
    std::cout << "hi" << std::endl;
    pthread_mutex_lock(&msg_ptr_->mutex);
    std::cout << "hey" << std::endl;
    int ret = pthread_cond_timedwait(&msg_ptr_->condvar, &msg_ptr_->mutex, &timeout);
    std::cout << "what" << std::endl;

    // we timed out!
    if (ret != 0) {
      pthread_mutex_unlock(&msg_ptr_->mutex);
      return false;
    }

    // copy data locally
    // std::memcpy(&msg->id, &msg_ptr_->id, sizeof(msg->id));
    // std::memcpy(&msg->pwm, &msg_ptr_->pwm, sizeof(msg->pwm));
    // std::memcpy(&msg->t, &msg_ptr_->t, sizeof(msg->t));

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
    // if (posix_fallocate(fd, 0, sizeof(msg_t)) != 0) return false;

    // Memory map the file with R/W permissions
    void * ptr = mmap(0, sizeof(msg_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

    // zero everything out
    // std::memset(msg_ptr_, 0, sizeof(msg_t));

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

int main(int argc, char const *argv[])
{

  Client client("data.bin");

  // keep track of how many messages were recvd
  uint32_t count = 0;
  uint32_t missed = 0;

  bool timedout = false;
  while (!timedout) {

    // time we started waiting at
    auto start = std::chrono::steady_clock::now();

    msg_t msg; msg.id = 1;
    timedout = !client.read(&msg);

    // make sure the message ids are synced
    if (msg.id == count) {
      count++;
    } else {
      missed += std::abs(static_cast<int32_t>(msg.id - count));
      count = msg.id;
    }

    std::cout << "count: " << count << std::endl;
  }



  std::cout << "received " << count-missed << " messages." << std::endl;
  std::cout << "missed " << missed << " messages." << std::endl;

  return 0;
}