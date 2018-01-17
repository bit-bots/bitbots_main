#ifndef _BITBOTS_TEST_UDPADAPTER
#define _BITBOTS_TEST_UDPADAPTER

#define DEF_BUFLEN 512

#include "netadapter.hpp"

class UdpAdapter : NetAdapter {
    
    public:
        UdpAdapter(int bufSize = DEF_BUFLEN);
        ~UdpAdapter();
        void sendMessage(string* msg, struct sockaddr_in* addr);
        Packet* recvMessage();
        int bindToPort(int port);
        int createSocket();
        int closeSocket();

    private:
        int bufSize;
        char* buf;

};

#endif