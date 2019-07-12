#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>

#include "msg.h"

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <time.h>
#include <pthread.h>

// ctrl+c handler
#include <cstdlib>
#include <csignal>

// interrupt signal (ctrl+c) handler
volatile sig_atomic_t stop = 0;
void handle_sigint(int s) { stop = 1; }

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

  Server server;

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

    server.send(&msg);

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