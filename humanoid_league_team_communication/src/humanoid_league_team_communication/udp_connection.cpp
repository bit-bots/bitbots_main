#include "humanoid_league_team_communication/udp_connection.h"

using namespace robocup;
using namespace humanoid;

UdpConnection::UdpConnection(int port) {
  // create socket
  if ((socketfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
    ROS_ERROR("TeamComm: UDP socket creation failed");
    exit(EXIT_FAILURE);
  }
  // enable broadcast
  int broadcastPermission = 1;
  if (setsockopt(socketfd, SOL_SOCKET, SO_BROADCAST, &broadcastPermission, sizeof(broadcastPermission)) == -1) {
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
  this->port = port;
}

UdpConnection::~UdpConnection() {
  close(socketfd);
}

void UdpConnection::send_data(Message send_msg) {
  // prepare message
  // convert to string
  std::string send_str;
  send_msg.SerializeToString(&send_str);
  // convert to char buffer
  char send_char[send_str.size()];
  std::size_t length = send_str.copy(send_char, send_str.size());

  // broadcast to all networks (adapted from mitecom)
  // get interfaces
  struct ifaddrs *interfaces;
  if (getifaddrs(&interfaces) == 0) {
    // go through all interfaces
    struct ifaddrs *p = interfaces;
    while (p) {
      // if we got an address for IP, use this interface
      if (p->ifa_addr && p->ifa_addr->sa_family == AF_INET) {
        // we make an intermediary cast to void* to prevent warnings about alignment requirements
        uint32_t interfaceAddress = ntohl(((struct sockaddr_in *) (void *) p->ifa_addr)->sin_addr.s_addr);
        uint32_t broadcastAddress = ((struct sockaddr_in *) (void *) p->ifa_broadaddr)->sin_addr.s_addr;

        // use all interfaces except the local one (we do not want to send to ourselves!)
        if (interfaceAddress > 0 && interfaceAddress != 0x7F000001) {
          struct sockaddr_in recipient = {0};
          recipient.sin_family = AF_INET;
          recipient.sin_port = htons(this->port);
          recipient.sin_addr.s_addr = broadcastAddress;
          sendto(socketfd, (void *) send_char, length, 0, (const struct sockaddr *) &recipient, sizeof recipient);
        }
      }
      p = p->ifa_next;
    }
    freeifaddrs(interfaces);
  }
}

Message UdpConnection::receive_data() {
  // receive data
  char receive_buffer[MAXSIZE];
  recv(socketfd, receive_buffer, sizeof(receive_buffer), 0);
  // convert to string
  std::string recv_str = receive_buffer;
  // convert to protobuf message
  Message recv_msg;
  recv_msg.ParseFromString(recv_str);

  return recv_msg;
}
