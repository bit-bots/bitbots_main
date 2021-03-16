#include "humanoid_league_team_communication/udp_connection.h"

using namespace robocup;
using namespace humanoid;

UdpConnection::UdpConnection(int port){
    // create socket
    if ((socketfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
        ROS_ERROR("TeamComm: UDP socket creation failed");
        exit(EXIT_FAILURE);
    }
    // enable broadcast
    int broadcastPermission = 1;
    if (setsockopt(socketfd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) == -1){
        ROS_ERROR("TeamComm: UDP broadcast enabling failed");
        exit(EXIT_FAILURE);
    }
    // bind socket
    struct sockaddr_in socket_addr;
    socket_addr.sin_family = AF_INET;
    socket_addr.sin_port = htons(port);
    socket_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(socketfd, (struct sockaddr *) &socket_addr, sizeof(socket_addr)) == -1) {
        ROS_ERROR("TeamComm: UDP socket binding failed");
        exit(EXIT_FAILURE);
    }
}

UdpConnection::~UdpConnection() {
    close(socketfd);
}

void UdpConnection::send_data(Message* send_msg) {
    // convert to string
    std::string send_str;
    send_msg->SerializeToString(&send_str);
    // convert to char buffer
    char send_char[send_str.size()];
    std::size_t length = send_str.copy(send_char, send_str.size());
    // send data
    send(socketfd, (void*) send_char, length, 0);
}

Message* UdpConnection::receive_data() {
    // receive data
    char receive_buffer[MAXSIZE];
    recv(socketfd, (void *) receive_buffer, MAXSIZE, 0);
    // convert to string
    std::string recv_str = receive_buffer;
    // convert to protobuf message
    Message* recv_msg;
    recv_msg->ParseFromString(recv_str);

    return recv_msg;
}