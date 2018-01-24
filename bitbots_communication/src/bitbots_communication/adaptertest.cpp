#include "ros/ros.h"
#include <iostream>
#include <cstdio>
#include <arpa/inet.h>
#include "udpadapter.hpp"
#include "packet.hpp"

using namespace std;

void handlePacket(Packet* packet) {

    printf("Received packet from %s:%d\n", inet_ntoa(packet->getAddr()->sin_addr),
            ntohs(packet->getAddr()->sin_port));
    printf("Data: %s\n" , packet->getMessage()->c_str());

    delete packet;
}

int main(int argc, char** args) {

    for(int i = 1; i < argc; i++) {
        cout << args[i] << endl;
    }

    cout << "Listening to 8888" << endl;

    UdpAdapter udpAdapter = UdpAdapter();

    udpAdapter.createSocket();
    udpAdapter.bindToPort(8888);

    while(true) {
        handlePacket(udpAdapter.recvMessage());
    }

    udpAdapter.closeSocket();
}