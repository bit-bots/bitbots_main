#ifndef HUMANOID_LEAGUE_TEAM_COMMUNICATION_UDP_CONNECTION_H
#define HUMANOID_LEAGUE_TEAM_COMMUNICATION_UDP_CONNECTION_H

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <ifaddrs.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <ros/console.h>
#include "robocup_extension.pb.h"
//#include <google/protobuf/message.h>


using namespace robocup;
using namespace humanoid;

class UdpConnection {
public:
    UdpConnection(int port);
    ~UdpConnection();
    void send_data(Message send_msg);
    Message* receive_data();

private:
    int socketfd;
    int port;
    const static size_t MAXSIZE=1000;
};


#endif //HUMANOID_LEAGUE_TEAM_COMMUNICATION_UDP_CONNECTION_H
