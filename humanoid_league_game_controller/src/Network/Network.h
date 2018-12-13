#ifndef NETWORK_H_
#define NETWORK_H_

#include "ros/ros.h"
#include <stdint.h>
#include <string>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <netdb.h>

typedef int SOCKET;
#define SOCKET_ERROR -1
#define INVALID_SOCKET -1

/**
 * A network connection for the exchange of serialized data
 */
class Network {
public:
    /**
     * Open a Port with the parameter given over the private construct method. Used no TCP connection.
     * @param address of the Target you want to connect with (IPv4).
     * @param port on with you want to connect with the target.
     * @param sourceAddress is the address of this robot/simulator (IPv4).
     * @param sourcePort is the Port of the source on the robot/simulator.
     */
    Network(uint32_t address, uint16_t port, uint32_t sourceAddress, uint16_t sourcePort);

    /**
     * Open a Port with the parameter given over the private construct method. Used no TCP connection.
     * SourcePort is 0 for the construction (Standard).
     * @param address of the Target you want to connect with (IPv4).
     * @param port on with you want to connect with the target.
     * @param sourceAddress is the address of this robot/simulator (IPv4).
     */
    Network(uint32_t address, uint16_t port, uint32_t sourceAddress);

    /**
     * Open a Port with the parameter given over the private construct method. Used no TCP connection.
     * SourcePort is 0 for the construction (Standard).
     * sourceAddress is 0 standart using for local.
     * @param address of the Target you want to connect with (IPv4).
     * @param port on with you want to connect with the target.
     */
    Network(uint32_t address, uint16_t port);

    /**
     * Open a Port with the parameter given over the private construct method.
     * SourcePort is 0 for the construction (Standard).
     * sourceAddress is 0 standard using for local.
     * @param address of the Target you want to connect with (IPv4).
     * @param port on with you want to connect with the target.
     * @param TCPMode can be set true if you want a real TCP Connection.
     */
    Network(uint32_t address, uint16_t port, bool TCPMode);

    /**
     * Destructor close socket under windows and reduce the mInstanceCounter.
     * If it is 0 WSACleanup is called to terminate winsocks.
     */
    ~Network();

    /**
     * Send Data to the address of this network (Broadcast or address).
     * @param data is the data that should be send to the address.
     * @param length of the data in bytes.
     */
    bool sendData(const char *data, size_t length);

    /**
     * Recive last message from the Network. Blocks until data is avaidable.
     * @param data reference where the data is stored.
     * @param maxLength of the data in bytes.
     * @return size of the data.
     */
    size_t receiveData(char *data, size_t maxLength);

    /**
     * Receive last message from the Network. Dont Blocks get
     * @param data reference where the data is stored.
     * @param maxLength of the data in bytes.
     * @param timeout is the time he waits while he is waiting for data.
     * @param addr is a pointer where the methods saved the address where the message comes from.
     * @param port is a pointer where the methods saved the port on the message is addressing.
     * @return size of the data if avaidable. If Timeout or error return 0.
     */
    size_t receiveData(char *data, size_t maxLength, struct timeval *timeout, uint32_t *addr, uint16_t *port);

    /**
     * Check wich address is the Local given as param in contructor to this network and returns it.
     * @return the ip of this machine.
     */
    uint32_t getLocalIP();

    /**
     * @return the destination ip address.
     */
    uint32_t getRemoteIP();

    /**
     * Get the interfaceIP, so basically checks the local ip by himself. No need for a network.
     * Basically can be used to identify ip address.
     * @return the local IP address. If there are multiple possibilities only the last one is returned.
     */
    uint32_t getInterfaceIP() const;

    /**
     * Set the ip address used as target for this network.
     * @param address of the target(IPv4). If the ip is 0 the address is set to broadcast.
     */
    void setRemoteIP(uint32_t address);

    /**
     * Print out the ip used for debugging.
     * @param ip is the ip-address that should be printed (ipv4).
     */
    static void printIP(uint32_t ip);

    /**
     * Find the ip address to the host name in this network.
     * @param host is the Name of the host.
     * @return ip of the host (ipv4)
     */
    static uint32_t resolveHost(std::string host);

    /**
     * Checks if the socket used by the network is valid.
     * @return positive true, negative false.
     */
    uint8_t isConnected() const;

    uint8_t isWifiConnected() const;

private:
    /**
     * Open a Port with the parameter given. Initial the (WIN)Sockets.
     * Add one to mInstanceCounter to count the connections.
     * @param address 		target address you want to connect with (IPv4). If address is 0 try a Broadcast.
     * @param port 			port on with you want to connect with the target.
     * @param sourceAddress 	the address of this robot/simulator (IPv4).
     * @param sourcePort 		the port of the source on the robot/simulator.
     * @param TCPMode 		can be set true if you want a real TCP Connection.
     */
    void construct(uint32_t address, uint16_t port, uint32_t sourceAddress, uint16_t sourcePort, bool TCPMode);

    /**
     * Initialize the sockets for windows.
     */
    static void initWinsocks();

    char *mInterfaceWifiName;
    SOCKET mSocket;                    //!< Socket of the network.
    struct sockaddr_in mSource;        //!< Source address(Local) of the network.
    struct sockaddr_in mDestination;    //!< Destination(Target) of the Network. If 0 -> Broadcast.

    static uint32_t mInstanceCounter;    //!< A static field how much Networks are used.
};

#endif /*NETWORK_H_*/
