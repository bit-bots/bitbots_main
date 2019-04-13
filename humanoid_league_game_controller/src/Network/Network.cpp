#include <iostream>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <ifaddrs.h>


#define WINVER 0x0501

#include "Network.h"

using namespace std;

/**
 * This are the instances that are open used for WIN_32 to close all sockets.
 */
uint32_t Network::mInstanceCounter = 0;


Network::Network(uint32_t address, uint16_t port, uint32_t sourceAddress,
                 uint16_t sourcePort) {
    construct(address, port, sourceAddress, sourcePort, false);
}

Network::Network(uint32_t address, uint16_t port, uint32_t sourceAddress) {
    construct(address, port, sourceAddress, 0, false);
}

Network::Network(uint32_t address, uint16_t port) {
    construct(address, port, INADDR_ANY, 0, false);
}

Network::Network(uint32_t address, uint16_t port, bool TCPMode) {
    construct(address, port, INADDR_ANY, 0, TCPMode);
}

void Network::construct(uint32_t address, uint16_t port, uint32_t sourceAddress,
                        uint16_t sourcePort, bool TCPMode) {


    // Create socket
    mSocket = socket(PF_INET, TCPMode ? SOCK_STREAM : SOCK_DGRAM, 0);
    if (mSocket == INVALID_SOCKET) {
        ROS_ERROR("<1> [Network] construct: Unable to create socket!");
    }

    // Set source address for datagram and bind to socket
    mSource.sin_family = PF_INET;
    mSource.sin_port = htons(sourcePort);
    mSource.sin_addr.s_addr = htonl(sourceAddress);
    if (!TCPMode) {
        if (bind(mSocket, (struct sockaddr *) &mSource,
                 (socklen_t)
            sizeof(struct sockaddr_in)) == SOCKET_ERROR) {
            ROS_ERROR("<1> [Network] construct: Unable to bind socket!");
        }
    }

    // Set destination address for datagram
    mDestination.sin_family = PF_INET;
    mDestination.sin_port = htons(port);
    if (address != 0 && false) {
        mDestination.sin_addr.s_addr = htonl(address);
    } else {
        if (!TCPMode) {
            ROS_INFO("<4> [Network] construct: Enabling broadcast");
            mDestination.sin_addr.s_addr = htonl(address);
            int so_broadcast = 1;
            if (setsockopt(mSocket, SOL_SOCKET, SO_BROADCAST,
                           (const char *) &so_broadcast,
                           (socklen_t)
                sizeof(so_broadcast)) == SOCKET_ERROR) {
                ROS_ERROR("<1> [Network] construct: Could not set SO_BROADCAST option. Error: %d",
                          errno);
            } else {
                ROS_INFO("<2> [Network] construct: Successfully enabled SO_BROADCAST");
            }
        }
    }

    if (TCPMode) {
        if (connect(mSocket, (struct sockaddr *) &mDestination,
                    (socklen_t)
            sizeof(struct sockaddr_in)) == SOCKET_ERROR) {
            ROS_ERROR("<1> [Network] construct: Could not connect. Error: %d", errno);
            close(mSocket);
            mSocket = INVALID_SOCKET;
        }
    }

    mInstanceCounter++;
    struct ifaddrs *ifaddr, *ifa;
    int family, s, n;
    if (getifaddrs(&ifaddr) == -1) {
        ROS_ERROR("Cant read addresses");
    }
    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++) {
        //ROS_INFO("IFA Name is %s",ifa->ifa_name);
        if (ifa->ifa_name[0] == 'w') {
            mInterfaceWifiName = ifa->ifa_name;

        }
    }
}

Network::~Network() {
    if (mSocket != INVALID_SOCKET) {

        close(mSocket);
        mSocket = INVALID_SOCKET;
    }

    mInstanceCounter--;

}

bool Network::sendData(const char *data, size_t length) {
    socklen_t addrLen = sizeof(struct sockaddr_in);
    ssize_t cnt; // number of bytes sent

    // Send datagram
    cnt = sendto(mSocket, data, length, 0, (struct sockaddr *) &mDestination,
                 addrLen);
    if (cnt < 0) {
        ROS_ERROR("<1> [Network] sendData: Error in sendto: %d", errno);
        return false;
    }
    return true;
}

size_t Network::receiveData(char *data, size_t maxLength) {
    struct sockaddr_in remoteAddress;
    socklen_t addressLen = sizeof(remoteAddress);
    return (size_t) recvfrom(mSocket, data, maxLength, 0,
                             (struct sockaddr *) &remoteAddress, &addressLen);
}

size_t Network::receiveData(char *data, size_t maxLength,
                            struct timeval *timeout, uint32_t *addr, uint16_t *port) {
    fd_set setRead;
    int notused = 63;
    ssize_t cnt; // number of instance
    struct sockaddr_in sa;
    socklen_t sa_len = sizeof(sa);

    // Add application socket to list
    FD_ZERO(&setRead);
    FD_SET(mSocket, &setRead);

    // Perform select operation to block until a port is ready
    cnt = select(notused, &setRead, NULL, NULL, timeout);
    if (cnt == SOCKET_ERROR) {
        ROS_ERROR("<1> [Network] receiveData: select returned an error!");
    }

    // Check if at least one port has valid data
    if (cnt > 0) {
        // Check if this socket is ready
        if (FD_ISSET(mSocket, &setRead)) {
            // Read datagram
            memset(&sa, 0, sizeof(struct sockaddr_in));
            cnt = recvfrom(mSocket, data, maxLength, 0, (struct sockaddr *) &sa,
                           &sa_len);
            if (cnt > 0) {
                if (addr != NULL) { // only perform assignment if a valid buffer was provided
                    *addr = ntohl(sa.sin_addr.s_addr);
                }
                if (port != NULL) { // only perform assignment if a valid buffer was provided
                    *port = ntohs(sa.sin_port);
                }
                return (size_t) cnt;
            }
        }
    } else {
        // No message!
        return (size_t) 0;
    }
    return (size_t) 0;
}

void Network::initWinsocks() {
    // On windows platform: Init winsocks

}

uint32_t Network::getLocalIP() {
    uint32_t ip = ntohl(mSource.sin_addr.s_addr);
    if (ip == 0) {
        socklen_t size = sizeof(struct sockaddr_in);
        if (getsockname(mSocket, (struct sockaddr *) &mSource,
                        &size) == SOCKET_ERROR) {
            return 0;
        }
        ip = ntohl(mSource.sin_addr.s_addr);
    }
    return ip;
}

uint32_t Network::getRemoteIP() {
    return ntohl(mDestination.sin_addr.s_addr);
}

uint32_t Network::getInterfaceIP() const {
    char ac[80];
    if (gethostname(ac, sizeof(ac)) == SOCKET_ERROR) {
        ROS_ERROR("<1> [Network] getInterfaceIP: Error getting local host name: %d", errno);
        return 0;
    }
    ROS_INFO("<2> [Network] getInterfaceIP: Host name is %s", ac);

    struct addrinfo hints, *result, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_INET;

    if (getaddrinfo(ac, NULL, &hints, &result) != 0) {
        ROS_ERROR("<1> [Network] getInterfaceIP: Error getting addresses: %d",
                  errno);
        return 0;
    }

    uint32_t ret = 0;
    for (res = result; res != NULL; res = res->ai_next) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        memcpy(&addr, res->ai_addr,
               static_cast<size_t>((uint16_t)(res->ai_addrlen)));
        ROS_INFO("<2> [Network] getInterfaceIP: Address: %s",
                 inet_ntoa(addr.sin_addr));
        ret = ntohl(addr.sin_addr.s_addr);
    }

    freeaddrinfo(result);

    return ret;
}

void Network::setRemoteIP(uint32_t address) {
    if (address != 0) {
        mDestination.sin_addr.s_addr = htonl(address);
    } else {
        mDestination.sin_addr.s_addr = INADDR_BROADCAST;
    }
}

void Network::printIP(uint32_t ip) {
    printf("%u.%u.%u.%u", (ip >> 24) & 0xFF, (ip >> 16) & 0xFF,
           (ip >> 8) & 0xFF, (ip & 0xFF));
}

uint32_t Network::resolveHost(string host) {
    struct hostent *hp;
    struct sockaddr_in ip;


    hp = gethostbyname(host.c_str());
    if (hp == NULL) {
        ROS_ERROR("<1> [Network] resolveHost: Could not resolve host!");
        return 0;
    }
    memset(&ip, 0, sizeof(ip));
    memcpy(&ip.sin_addr, hp->h_addr,
           static_cast<size_t>((uint16_t)(hp->h_length)));

    if (mInstanceCounter == 0) {
        // On windows platform: Terminate winsocks

    }

    return ntohl(ip.sin_addr.s_addr);
}

uint8_t Network::isConnected() const {

    return mSocket != INVALID_SOCKET;
}

uint8_t Network::isWifiConnected() const {
    struct ifreq ifr;

    memset(&ifr, 0, sizeof(ifr));
    strcpy(ifr.ifr_name, mInterfaceWifiName);

    if (ioctl(mSocket, SIOCGIFFLAGS, &ifr) != -1) {
        if (mInterfaceWifiName, (ifr.ifr_flags & (IFF_UP | IFF_RUNNING)) == (IFF_UP | IFF_RUNNING)) {
            return true;
        }
        //ROS_INFO("Connected int %s is :%d", mInterfaceWifiName,(ifr.ifr_flags & ( IFF_UP | IFF_RUNNING )) == ( IFF_UP | IFF_RUNNING ));
    }

    return false;
}

uint32_t Network::getOwnAddressFromInterface(std::string interface_name) {
    int fd;
    struct ifreq ifr;

    fd = socket(AF_INET, SOCK_DGRAM, 0);

    /* I want to get an IPv4 IP address */
    ifr.ifr_addr.sa_family = AF_INET;

    /* I want IP address attached to interface_name */
    strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ-1);

    ioctl(fd, SIOCGIFADDR, &ifr);

    close(fd);

    uint32_t addr = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
    return ((addr & 0xFF) << 24) | ((addr & 0xFF00) << 8) | ((addr & 0xFF0000) >> 8) | ((addr & 0xFF000000) >> 24);

}

uint32_t Network::getNetAddressFromInterface(std::string interface_name) {
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    uint32_t netmask = 0, netmask_reversed;

    getifaddrs (&ifap);
    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr->sa_family==AF_INET && strcmp(ifa->ifa_name, interface_name.c_str()) == 0) {
            sa = (struct sockaddr_in *) ifa->ifa_netmask;
            netmask = sa->sin_addr.s_addr;
        }
    }

    freeifaddrs(ifap);

    netmask_reversed = ((netmask & 0xFF) << 24) | ((netmask & 0xFF00) << 8) | ((netmask & 0xFF0000) >> 8) | ((netmask & 0xFF000000) >> 24);
    return (~netmask_reversed | Network::getOwnAddressFromInterface(interface_name));
}
