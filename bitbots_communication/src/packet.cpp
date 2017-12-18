#include "packet.hpp"

Packet::Packet(struct sockaddr_in* addr, string* message) {
    Packet::addr = new sockaddr_in;
    Packet::message = message;
    memcpy(Packet::addr, addr, sizeof(sockaddr_in));
}

Packet::~Packet() {
    delete addr;
    delete message;
}

struct sockaddr_in* Packet::getAddr() {
    return addr;
}

string* Packet::getMessage() {
    return message;
}