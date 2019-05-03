#include <iostream>
#include <string>
#include <cstring>

#include "msg.h"

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <time.h>
#include <pthread.h>

class Server
{
public:
  Server()
  {
    // initialize shared memory
    initSHM();

    // initialize the mutex and condvar to synchronize
    // across process boundaries for concurrency-safe
    // shared memory read and write.
    initMutex();
  }

  ~Server()
  {
    // n.b. order matters
    deinitMutex();
    deinitSHM();
  }

  void send(const msg_t* msg)
  {
    pthread_mutex_lock(&msg_ptr_->mutex);

    // copy data to memory-mapped file
    msg_ptr_->id = msg->id;
    std::memcpy(&msg_ptr_->id, &msg->id, sizeof(msg->id));
    std::memcpy(&msg_ptr_->pwm, &msg->pwm, sizeof(msg->pwm));
    std::memcpy(&msg_ptr_->t, &msg->t, sizeof(msg->t));

    // signal client to read data
    pthread_cond_signal(&msg_ptr_->condvar);

    pthread_mutex_unlock(&msg_ptr_->mutex);
  }

private:
  int shmid_; ///< shared memory segment id
  msg_t * msg_ptr_ = nullptr;

  bool initSHM()
  {
    // generates an arbitrary long based on stat info of file
    key_t key = ftok("shmem_server", 'A');

    // create and connect to a shared memory segment (check with 'ipcs -m')
    shmid_ = shmget(key, sizeof(msg_t), IPC_CREAT | /*IPC_EXCL |*/ S_IRUSR | S_IWUSR);

    // attach to the shared memory segment
    void * ptr = shmat(shmid_, 0, 0);
    msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

    std::cout << "shmid: " << shmid_ << std::endl;

    return true;
  }

  // --------------------------------------------------------------------------

  void deinitSHM()
  {
    // detach
    shmdt(reinterpret_cast<void *>(msg_ptr_));

    // destroy
    shmctl(shmid_, IPC_RMID, NULL);
  }

  // --------------------------------------------------------------------------

  void deinitMutex()
  {
    pthread_mutex_destroy(&msg_ptr_->mutex);
    // If client is killed before server, this will block due to undef behavior
    // pthread_cond_destroy(&msg_ptr_->condvar);
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

  Server server;

  constexpr int MSG_COUNT = 1000;

  for (int i=0; i<MSG_COUNT; ++i) {
    msg_t msg;
    msg.id = i;
    for (int j=0; j<NUM_PWM; ++j) {
      msg.pwm[j] =  i + j*0.1f + 0.01f;
    }
    clock_gettime(CLOCK_MONOTONIC, &msg.t);

    server.send(&msg);

    std::cout << "sent " << msg.id << std::endl;

    for (int j=0; j<1888008; ++j);

  }

  std::cout << "done" << std::endl;

  return 0;
}