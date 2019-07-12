#pragma once

#include "msg.h"

class Client
{
public:
  Client() = default;
  ~Client() = default;
    
  virtual bool read(msg_t * msg) = 0;
};

class Server
{
public:
  Server() = default;
  ~Server() = default;
    
  virtual void send(const msg_t * msg) = 0;
};
