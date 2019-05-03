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
#include <sys/ipc.h>
#include <sys/shm.h>

class Client
{
public:
  Client()
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
    pthread_mutex_lock(&msg_ptr_->mutex);
    int ret = pthread_cond_timedwait(&msg_ptr_->condvar, &msg_ptr_->mutex, &timeout);

    // we timed out!
    if (ret != 0) {
      pthread_mutex_unlock(&msg_ptr_->mutex);
      // if we timed out, destroy so that server can create a new
      // segment (if server has "IPC_EXCL" shmget flag).
      // shmctl(shmid_, IPC_RMID, NULL);
      return false;
    }

    // copy data locally
    msg->id = msg_ptr_->id;
    std::memcpy(&msg->id, &msg_ptr_->id, sizeof(msg->id));
    std::memcpy(&msg->pwm, &msg_ptr_->pwm, sizeof(msg->pwm));
    std::memcpy(&msg->t, &msg_ptr_->t, sizeof(msg->t));

    pthread_mutex_unlock(&msg_ptr_->mutex);
    return true;
  }

private:
  int shmid_;
  msg_t * msg_ptr_ = nullptr;

  bool init()
  {
    // generates an arbitrary long based on stat info of file
    key_t key = ftok("shmem_server", 'A');

    // create and connect to a shared memory segment (check with 'ipcs -m')
    shmid_ = shmget(key, sizeof(msg_t), IPC_CREAT | S_IRUSR | S_IWUSR);

    // attach to the shared memory segment
    void * ptr = shmat(shmid_, 0, 0);
    msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

    std::cout << "shmid: " << shmid_ << std::endl;

    return true;
  }

  // --------------------------------------------------------------------------

  void deinit()
  {
    // detach
    shmdt(reinterpret_cast<void *>(msg_ptr_));

    // destroy (server's job)
    // shmctl(shmid_, IPC_RMID, NULL);
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

  Client client;

  // keep track of how many messages were recvd
  uint32_t count = 0;
  uint32_t id = 0;
  uint32_t missed = 0;

  bool first = true;

  while (true) {

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
    std::cout << "count (id): " << count << " (" << id << ") [" << usec << " usec]" << std::endl;

    id++;
    count++;
  }

  std::cout << "received " << count-missed << " messages." << std::endl;
  std::cout << "missed " << missed << " messages." << std::endl;

  return 0;
}