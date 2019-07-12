#include <iostream>
#include <string>
#include <cstring>
#include <stdexcept>
#include <exception>

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

#include <time.h>
#include <pthread.h>

#include "mmap.h"

MmapClient::MmapClient(const std::string path)
: path_(path)
{
  if (!init()) {
    std::throw_with_nested(std::runtime_error("could not initialize"));
  }
}

// ----------------------------------------------------------------------------

MmapClient::~MmapClient()
{
  deinit();
}

// ----------------------------------------------------------------------------

bool MmapClient::read(msg_t * msg)
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

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool MmapClient::init()
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

// ----------------------------------------------------------------------------

void MmapClient::deinit()
{
  munmap(reinterpret_cast<void *>(msg_ptr_), sizeof(msg_t));
}


// ============================================================================
// ============================================================================
// ============================================================================

MmapServer::MmapServer(const std::string path)
: path_(path)
{
  // initialize memory-mapped file 
  initMMF();

  // initialize the mutex and condvar to synchronize
  // across process boundaries for concurrency-safe
  // memory-mapped file read and write.
  initMutex();
}

// ----------------------------------------------------------------------------

MmapServer::~MmapServer()
{
  // n.b. order matters
  deinitMutex();
  deinitMMF();
}

// ----------------------------------------------------------------------------

void MmapServer::send(const msg_t* msg)
{
  pthread_mutex_lock(&msg_ptr_->mutex);

  // copy data to memory-mapped file
  std::memcpy(&msg_ptr_->id, &msg->id, sizeof(msg->id));
  std::memcpy(&msg_ptr_->pwm, &msg->pwm, sizeof(msg->pwm));
  std::memcpy(&msg_ptr_->t, &msg->t, sizeof(msg->t));

  // signal client to read data
  int ret = pthread_cond_signal(&msg_ptr_->condvar);

  pthread_mutex_unlock(&msg_ptr_->mutex);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool MmapServer::initMMF()
{
  // Open the file with R/W permissions, create if it does not exist
  const int fd = open(path_.c_str(), O_RDWR | O_CREAT, 0600);
  if (fd == -1) return false;

  // Stretch the file size to the desired size
  if (posix_fallocate(fd, 0, sizeof(msg_t)) != 0) return false;

  // Memory map the file with R/W permissions
  void * ptr = mmap(0, sizeof(msg_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  msg_ptr_ = reinterpret_cast<msg_t *>(ptr);

  // zero everything out
  std::memset(msg_ptr_, 0, sizeof(msg_t));

  std::cout << "created " << path_ << std::endl;

  close(fd);
  return true;
}

// ----------------------------------------------------------------------------

void MmapServer::deinitMMF()
{
  munmap(reinterpret_cast<void *>(msg_ptr_), sizeof(msg_t));
}

// ----------------------------------------------------------------------------

void MmapServer::deinitMutex()
{
  pthread_mutex_destroy(&msg_ptr_->mutex);
  // If client is killed before server, this will block due to undef behavior
  // pthread_cond_destroy(&msg_ptr_->condvar);
}
  
// ----------------------------------------------------------------------------

void MmapServer::initMutex()
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
