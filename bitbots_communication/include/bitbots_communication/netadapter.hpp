#ifndef _BITBOTS_TEST_NETADAPTER
#define _BITBOTS_TEST_NETADAPTER

#define STATE_READY 1
#define STATE_NO_SOCKET 0

#include <cstdio>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <string>

#include "packet.hpp"

using namespace std;

class NetAdapter {
    public:
        NetAdapter();
        virtual ~NetAdapter();
        virtual void sendMessage(string* msg, struct sockaddr_in* addr);
        virtual Packet* recvMessage();
        virtual int bindToPort(int port);
        virtual int createSocket();
        virtual int closeSocket();

    protected:
        int socketId;
        int state;
        int sockAddrLength;
        struct sockaddr_in* localAddr;
};

struct sockaddr_in* createAddress(string* ip, int port);
struct sockaddr_in* createAddress(int port);

#endif