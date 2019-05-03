#include <iostream>
#include <string>
#include <cstring>

#include "msg.h"

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

#include <time.h>
#include <pthread.h>

class Server
{
public:
  Server(const std::string path)
  : path_(path)
  {
    // initialize memory-mapped file 
    initMMF();

    // initialize the mutex and condvar to synchronize
    // across process boundaries for concurrency-safe
    // memory-mapped file read and write.
    initMutex();
  }

  ~Server()
  {
    // n.b. order matters
    deinitMutex();
    deinitMMF();
  }

  void send(const msg_t* msg)
  {
    pthread_mutex_lock(&msg_ptr_->mutex);

    // copy data to memory-mapped file
    // std::memcpy(&msg_ptr_->id, &msg->id, sizeof(msg->id));
    // std::memcpy(&msg_ptr_->pwm, &msg->pwm, sizeof(msg->pwm));
    // std::memcpy(&msg_ptr_->t, &msg->t, sizeof(msg->t));

    // signal client to read data
    pthread_cond_signal(&msg_ptr_->condvar);

    pthread_mutex_unlock(&msg_ptr_->mutex);
  }

private:
  const std::string path_;
  msg_t * msg_ptr_ = nullptr;

  bool initMMF()
  {
    // Open the file with R/W permissions, create if it does not exist
    const int fd = open(path_.c_str(), O_RDWR | O_CREAT, 0600);
    if (fd == -1) return false;

    // Stretch the file size to the desired size
    // if (posix_fallocate(fd, 0, sizeof(msg_t)) != 0) return false;

    // Memory map the file with R/W permissions
    void * ptr = mmap(0, sizeof(msg_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

    close(fd);
    return true;
  }

  // --------------------------------------------------------------------------

  void deinitMMF()
  {
    munmap(reinterpret_cast<void *>(msg_ptr_), sizeof(msg_t));
  }

  // --------------------------------------------------------------------------

  void deinitMutex()
  {
    pthread_mutex_destroy(&msg_ptr_->mutex);
    pthread_cond_destroy(&msg_ptr_->condvar);
  }
    
  // --------------------------------------------------------------------------

  void initMutex()
  {
    // n.b.: this assumes that msg_ptr_ already points at allocated memory
    //       (e.g., from initMMF).
    //       Also, only the server will init and destroy mutex/condvar

    // mutex setup
    pthread_mutexattr_t mattr;
    pthread_mutexattr_init(&mattr);
    pthread_mutexattr_setpshared(&mattr, PTHREAD_PROCESS_SHARED);
    pthread_mutex_init(&msg_ptr_->mutex, &mattr);

    // conditional variable setup
    pthread_condattr_t cvattr;
    pthread_condattr_init(&cvattr);
    pthread_condattr_setpshared(&cvattr, PTHREAD_PROCESS_SHARED);
    pthread_cond_init(&msg_ptr_->condvar, &cvattr);

  }
};

// ============================================================================
// ============================================================================

int main(int argc, char const *argv[])
{

  Server server("data.bin");

  constexpr int MSG_COUNT = 1000;

  for (int i=0; i<MSG_COUNT; ++i) {
    msg_t msg;
    msg.id = i;
    for (int j=0; j<NUM_PWM; ++j) {
      msg.pwm[j] =  i + j*0.1f + 0.01f;
    }
    clock_gettime(CLOCK_MONOTONIC, &msg.t);

    server.send(&msg);

    for (int j=0; j<1888008; ++j);
  }


  return 0;
}