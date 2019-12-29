#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <exception>

// POSIX shared memory
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <time.h>
#include <pthread.h>

#include "shmem.h"

ShmemClient::ShmemClient()
{
  if (!init()) {
    std::throw_with_nested(std::runtime_error("could not initialize"));
  }
}

// ----------------------------------------------------------------------------

ShmemClient::~ShmemClient()
{
  deinit();
}

// ----------------------------------------------------------------------------

bool ShmemClient::read(msg_t * msg)
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

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool ShmemClient::init()
{
  // generates an arbitrary long based on stat info of file
  key_t key = ftok("shmem_server", 'A');

  // connect to a shared memory segment (check with 'ipcs -m')
  connectToShmem(key);

  // attach to the shared memory segment
  void * ptr = shmat(shmid_, 0, 0);
  msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

  // mark for destruction, also prevent others from attaching
  shmctl(shmid_, IPC_RMID, NULL);

  std::cout << "shmid: " << shmid_ << std::endl;

  return true;
}

// ----------------------------------------------------------------------------

void ShmemClient::connectToShmem(const key_t key)
{
  while (shmid_ == -1) {
    // n.b., we let the server create the memory
    shmid_ = shmget(key, sizeof(msg_t), /*IPC_CREAT |*/ S_IRUSR | S_IWUSR);

    if (shmid_ == -1) {
      std::cout << "Waiting on server to initialize shared memory..." << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

// ----------------------------------------------------------------------------

void ShmemClient::deinit()
{
  // detach
  shmdt(reinterpret_cast<void *>(msg_ptr_));

  // destroy (server's job)
  // shmctl(shmid_, IPC_RMID, NULL);
}

// ============================================================================
// ============================================================================
// ============================================================================

ShmemServer::ShmemServer()
{
  // initialize shared memory
  initSHM();

  // initialize the mutex and condvar to synchronize
  // across process boundaries for concurrency-safe
  // shared memory read and write.
  initMutex();
}

// ----------------------------------------------------------------------------

ShmemServer::~ShmemServer()
{
  // n.b. order matters
  deinitMutex();
  deinitSHM();
}

// ----------------------------------------------------------------------------

void ShmemServer::send(const msg_t* msg)
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

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool ShmemServer::initSHM()
{
  // generates an arbitrary long based on stat info of file
  key_t key = ftok("shmem_server", 'A');

  // create and connect to a shared memory segment (check with 'ipcs -m')
  shmid_ = shmget(key, sizeof(msg_t), IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);

  // attach to the shared memory segment
  void * ptr = shmat(shmid_, 0, 0);
  msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

  std::cout << "shmid: " << shmid_ << std::endl;

  return true;
}

// ----------------------------------------------------------------------------

void ShmemServer::deinitSHM()
{
  // detach
  shmdt(reinterpret_cast<void *>(msg_ptr_));

  // destroy
  // shmctl(shmid_, IPC_RMID, NULL);
}

// ----------------------------------------------------------------------------

void ShmemServer::deinitMutex()
{
  pthread_mutex_destroy(&msg_ptr_->mutex);
  // If client is killed before server, this will block due to undef behavior
  // pthread_cond_destroy(&msg_ptr_->condvar);
}
  
// ----------------------------------------------------------------------------

void ShmemServer::initMutex()
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
