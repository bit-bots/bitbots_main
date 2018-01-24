#ifndef _BITBOTS_TEST_PACKET
#define _BITBOTS_TEST_PACKET

#include <string>
#include <string.h>
#include <arpa/inet.h>

using namespace std;

class Packet {
    public:
        Packet(struct sockaddr_in* addr, string* message);
        ~Packet();
        struct sockaddr_in* getAddr();
        string* getMessage();
    protected:
        struct sockaddr_in* addr;
        string* message;
};

#endif