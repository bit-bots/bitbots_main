#include "udpadapter.hpp"

UdpAdapter::UdpAdapter(int bufSize) {
    this->bufSize = bufSize;
    buf = new char[bufSize];
}

UdpAdapter::~UdpAdapter() {
    delete buf;
    delete localAddr;
}

int UdpAdapter::bindToPort(int port) {
    if(state == STATE_READY) {
        localAddr = createAddress(port);
        return bind(socketId, (struct sockaddr*) localAddr, sockAddrLength);
    }

    return -1;
}

int UdpAdapter::createSocket() {
    if(state == STATE_NO_SOCKET) {
        int ret = (socketId = socket(AF_INET, SOCK_DGRAM, 0));

        if(ret > -1) {
            state = STATE_READY;
        }

        return ret;
    } else {
        return socketId;
    }
}

void UdpAdapter::sendMessage(string* msg, struct sockaddr_in* addr) {
    if(state == STATE_READY) {
        sendto(socketId, msg->data(), msg->length(), 0,
            (struct sockaddr*) addr, (socklen_t) sockAddrLength);
    }
}

int UdpAdapter::closeSocket() {
    if(state == STATE_READY) {
        state = STATE_NO_SOCKET;
        close(socketId);
        socketId = -1;
        return 0;
    } else {
        return -1;
    }
}

Packet* UdpAdapter::recvMessage() {
    if(state == STATE_READY) {
        struct sockaddr_in recvAddr = sockaddr_in();
        memset(&recvAddr, 0, sockAddrLength);
        memset(buf, '\0', bufSize);
        recvfrom(socketId, buf, bufSize, 0, (sockaddr*) &recvAddr, (socklen_t*) &sockAddrLength);

        string* data = new string(buf);

        return new Packet(&recvAddr, data);
    }
    
    else {
        return NULL;
    }
}