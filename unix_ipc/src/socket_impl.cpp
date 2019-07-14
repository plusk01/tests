#include <iostream>
#include <string>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <exception>

#include <time.h>

#include "socket.h"

SocketClient::SocketClient()
{
  if (!init()) {
    std::throw_with_nested(std::runtime_error("could not initialize"));
  }
}

// ----------------------------------------------------------------------------

SocketClient::~SocketClient()
{
  deinit();
}

// ----------------------------------------------------------------------------

bool SocketClient::read(msg_t * msg)
{
  return false;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool SocketClient::init()
{
  return true;
}

// ----------------------------------------------------------------------------

void SocketClient::deinit()
{

}

// ============================================================================
// ============================================================================
// ============================================================================

SocketServer::SocketServer(std::string path)
: socketPath_(path)
{
  if (!init()) {
    std::throw_with_nested(std::runtime_error("could not initialize"));
  }
}

// ----------------------------------------------------------------------------

SocketServer::~SocketServer()
{
  deinit();
}

// ----------------------------------------------------------------------------

void SocketServer::send(const msg_t* msg)
{

}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

bool SocketServer::init()
{
  socket_ = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (socket_ == -1) return false;

  // server bind address
  localAddr_.sun_family = AF_UNIX;
  strcpy(localAddr_.sun_path, socketPath_.cstr());
  unlink(localAddr_.sun_path); // rm if already exists
  int len = strlen(localAddr_.sun_path) + sizeof(local.sun_family);

  // attempt to bind
  if (bind(socket_, (sockaddr *)&localAddr_, len) == -1) return false;

  // listen for incoming connections
  // if there are more than _ waiting to connect, refuse client connections
  if (listen(socket_, MAXCONN) == -1) return false;

  //
  // epoll -- async event handling (https://codereview.stackexchange.com/q/98558)
  //

  epoll_ = epoll_create(MAXCONN);

  return true;
}

// ----------------------------------------------------------------------------

void SocketServer::deinit()
{
  close(socket_);
}
