// Multicast Server
// written for LINUX
// Version 0.0.2
//
// Change: IP_MULTICAST_LOOP : Enable / Disable loopback for outgoing messages
// 
// Compile : gcc -o server server.c
//
// This code has NOT been tested
// 

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define MAXBUFSIZE 65536 // Max UDP Packet size is 64 Kbyte

int main()
{
   int sock, status, socklen;
   char buffer[MAXBUFSIZE];
   struct sockaddr_in saddr;
   struct ip_mreq imreq;

   // set content of struct saddr and imreq to zero
   memset(&saddr, 0, sizeof(struct sockaddr_in));
   memset(&imreq, 0, sizeof(struct ip_mreq));

   // open a UDP socket
   sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
   if ( sock < 0 )
     perror("Error creating socket"), exit(0);

   saddr.sin_family = PF_INET;
   saddr.sin_port = htons(4096); // listen on port 4096
   saddr.sin_addr.s_addr = htonl(INADDR_ANY); // bind socket to any interface
   status = bind(sock, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));

   if ( status < 0 )
     perror("Error binding socket to interface"), exit(0);

   imreq.imr_multiaddr.s_addr = inet_addr("226.0.0.1");
   imreq.imr_interface.s_addr = INADDR_ANY; // use DEFAULT interface

   // JOIN multicast group on default interface
   status = setsockopt(sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, 
	      (const void *)&imreq, sizeof(struct ip_mreq));

   socklen = sizeof(struct sockaddr_in);

   // receive packet from socket
   status = recvfrom(sock, buffer, MAXBUFSIZE, 0, 
		     (struct sockaddr *)&saddr, &socklen);

   printf("Received: %s\n", buffer, MAXBUFSIZE);

   // shutdown socket
   shutdown(sock, 2);
   // close socket
   close(sock);

   return 0;
}