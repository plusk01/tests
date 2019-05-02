#include <iostream>
#include <string>

#include "msg.h"

// POSIX memory mapping
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>

class Client
{
public:
  Client(const std::string path) : path_(path) { init(); }
  ~Client() { deinit(); }

private:
  const std::string path_;
  msg_t * msg_ptr_ = nullptr;

  bool init()
  {
    // Open the file with R/W permissions, create if it does not exist
    const int fd = open(path_.c_str(), O_RDWR | O_CREAT, 0600);
    if (fd == -1) return false;

    // Stretch the file size to the desired size (size-1 for write call)
    if (lseek(fd, sizeof(msg_t)-1, SEEK_SET) == -1) return false;

    // Write to the end of the file to commit to the new size
    if (write(fd, "", 1) == -1) return false;

    // TODO: Replace lseek and write with posix_fallocate?

    // Memory map the file with R/W permissions
    void * ptr = mmap(0, sizeof(msg_t), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    msg_ptr_ = reinterpret_cast<msg_t *>(ptr);
  }

  // --------------------------------------------------------------------------

  void deinit()
  {
    munmap(reinterpret_cast<void *>(msg_ptr_), sizeof(msg_t));
  }
    
};

// ============================================================================
// ============================================================================

int main(int argc, char const *argv[])
{

  Client client("data.bin");


  return 0;
}