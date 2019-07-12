#pragma once

#include "abstract.h"

class ShmemClient : public Client
{
public:
  ShmemClient();
  ~ShmemClient();

  bool read(msg_t * msg) override;

private:
  int shmid_;
  msg_t * msg_ptr_ = nullptr;

  bool init();
  void deinit();
};

// ============================================================================
// ============================================================================
// ============================================================================

class ShmemServer : public Server
{
public:
  ShmemServer();
  ~ShmemServer();

  void send(const msg_t* msg) override;

private:
  int shmid_; ///< shared memory segment id
  msg_t * msg_ptr_ = nullptr;

  bool initSHM();
  void deinitSHM();
  void deinitMutex();
  void initMutex();
};
