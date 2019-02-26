#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <time.h>
#include <string.h>
#include <stdio.h>

#include <iostream>
#include <string>

using namespace std;

#define HELLO_PORT 12345
#define HELLO_GROUP "225.0.0.37"
#define MSGBUFSIZE 256

int main(int argc, char *argv[])
{
    string source_iface;
    string group(HELLO_GROUP);
    int port(HELLO_PORT);

    if (!(argc < 2)) group = argv[1];
    if (!(argc < 3)) port = atoi(argv[2]);
    if (!(argc < 4)) source_iface = argv[3];

    cout << "group: " << group << " port: " << port << " source_iface: " << source_iface << endl;

    int fd;
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket");
        exit(1);
    }

    u_int yes = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
    {
        perror("Reusing ADDR failed");
        exit(1);
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = (source_iface.empty() ? htonl(INADDR_ANY) : inet_addr(source_iface.c_str()));

if (bind(fd,(struct sockaddr *)&addr, sizeof(addr)) < 0)
{
    perror("bind");
    exit(1);
}

struct ip_mreq mreq;
mreq.imr_multiaddr.s_addr = inet_addr(group.c_str());
mreq.imr_interface.s_addr = (source_iface.empty() ? htonl(INADDR_ANY) : inet_addr(source_iface.c_str()));

if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
{
    perror("setsockopt");
    exit(1);
}

socklen_t addrlen;
int nbytes;
char msgbuf[MSGBUFSIZE];

while (1)
{
    memset(&msgbuf, 0, MSGBUFSIZE);

    addrlen = sizeof(addr);
    if ((nbytes = recvfrom(fd, msgbuf, MSGBUFSIZE, 0, (struct sockaddr *)&addr, &addrlen)) < 0)
    {
        perror("recvfrom");
        exit(1);
    }
   cout.write(msgbuf, nbytes);
    cout.flush();
}

return 0;
}
