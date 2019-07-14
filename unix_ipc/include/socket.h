#pragma once

#include <string>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "abstract.h"

class SocketClient : public Client
{
public:
  SocketClient();
  ~SocketClient();

  bool read(msg_t * msg) override;

private:

  bool init();
  void deinit();
};

// ============================================================================
// ============================================================================
// ============================================================================

class SocketServer : public Server
{
public:
  SocketServer(std::string path);
  ~SocketServer();

  void send(const msg_t* msg) override;

private:
  static constexpr MAXCONN = 5; ///< maximum client connections to listen for
  int socket_; ///< UNIX socket file descriptor
  sockaddr_un localAddr_; ///< Local address info for socket server to listen
  std::string socketPath_; ///< UNIX path to socket

  int epoll_; ///< fd for epoll instance

  bool init();
  void deinit();
};
