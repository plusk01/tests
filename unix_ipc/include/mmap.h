#pragma once

#include <string>

#include "abstract.h"

class MmapClient : public Client
{
public:
  MmapClient(const std::string path);
  ~MmapClient();

  bool read(msg_t * msg) override;

private:
  const std::string path_;
  msg_t * msg_ptr_ = nullptr;

  bool init();
  void deinit();
};

// ============================================================================
// ============================================================================
// ============================================================================

class MmapServer : public Server
{
public:
  MmapServer(const std::string path);
  ~MmapServer();

  void send(const msg_t* msg) override;

private:
  const std::string path_;
  msg_t * msg_ptr_ = nullptr;

  bool initMMF();
  void deinitMMF();
  void deinitMutex();
  void initMutex();
};
