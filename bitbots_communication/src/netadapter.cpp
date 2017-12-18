#include "netadapter.hpp"

NetAdapter::NetAdapter() {
    sockAddrLength = sizeof(struct sockaddr_in);
    state = STATE_NO_SOCKET;
}

NetAdapter::~NetAdapter() {
    delete localAddr;
}

void NetAdapter::sendMessage(string* msg, struct sockaddr_in* addr) {}
Packet* NetAdapter::recvMessage(){return NULL;}
int NetAdapter::bindToPort(int port){return 0;}
int NetAdapter::createSocket(){return 0;}
int NetAdapter::closeSocket() {return 0;}

struct sockaddr_in* createAddress(int port) {
    struct sockaddr_in* addr = new sockaddr_in;
    memset((char*) addr, 0, sizeof(sockaddr_in));

    addr->sin_family = AF_INET;
    addr->sin_port = htons(port);
    addr->sin_addr.s_addr = htonl(INADDR_ANY);

    return addr;
}

struct sockaddr_in* createAddress(string* ip, int port) {
    struct sockaddr_in* addr = createAddress(port);

    if(inet_aton(ip->data(), &addr->sin_addr) == 0) {
        cout << "inet_aton failed: " << ip->c_str() << endl;
        return NULL;
    }

    return addr;
}