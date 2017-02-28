

#include <arpa/inet.h>
#include <sys/socket.h>
#include <ifaddrs.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>


/*------------------------------------------------------------------------------------------------*/

/**
 ** Open a listening UDP socket on the given port.
 **
 ** @param port The UDP port to listen on
 **
 ** @return -1 on error, otherwise the socket descriptor
 */

int mitecom_open(int port) {
	int sock;

	// create socket
	if ((sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
		fprintf(stderr, "Failure creating socket\n");
		return -1;
	}

	// enable broadcast
	int broadcastPermission = 1;
	setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (void *) &broadcastPermission, sizeof(broadcastPermission));

	// make socket non-blocking
	int fcntl_flags = fcntl(sock, F_GETFL, 0);
	fcntl(sock, F_SETFL, fcntl_flags | O_NONBLOCK);

	// construct sockaddr_in structure
	struct sockaddr_in sa = { 0 };
	sa.sin_family      = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY);
	sa.sin_port        = htons(port);

	// bind socket
	unsigned int saLen = sizeof(sa);
	if ( bind(sock, (struct sockaddr *) &sa, saLen) < 0 ) {
		fprintf(stderr, "Failure binding socket to port %d\n", port);
		close(sock);
		return -1;
	}

	printf("Opened mixed team communication port %d\n", port);
	return sock;
}


/*------------------------------------------------------------------------------------------------*/

/** Receive a UDP  package.
 **
 ** @param[in]  sock           The socket to receive the package from
 ** @param[out] buffer         Buffer to fill with the received UDP package
 ** @param[in]  bufferSize     Size of buffer
 **
 ** @return the number of bytes received, or -1 on error
 */

ssize_t mitecom_receive(int sock, void* buffer, unsigned int bufferSize) {
	struct sockaddr_in remoteAddress;
	socklen_t addressLen = sizeof(remoteAddress);
	return recvfrom(sock, buffer, bufferSize, 0, (struct sockaddr *)&remoteAddress, &addressLen);
}


/*------------------------------------------------------------------------------------------------*/

/** Send a broadcast message from the given socket to a port.
 **
 ** @param sock       The outgoing (local) socket
 ** @param port       The port to broadcast to
 ** @param data       The data to send
 ** @param dataLength The length (in bytes) of the data to send
 */

void mitecom_broadcast(int sock, int port, const void* data, uint32_t dataLength) {
	struct ifaddrs * ifap;
	if (getifaddrs(&ifap) == 0) {
		// go through all interfaces
		struct ifaddrs * p = ifap;
		while (p) {
			// if we got an address for IP, use this interface
			if (p->ifa_addr && p->ifa_addr->sa_family == AF_INET) {
				// we make an intermediary cast to void* to prevent warnings about alignment requirements
				uint32_t interfaceAddress = ntohl(((struct sockaddr_in *)(void*)p->ifa_addr)->sin_addr.s_addr);
				uint32_t broadcastAddress = ((struct sockaddr_in *)(void*)p->ifa_broadaddr)->sin_addr.s_addr;

#if defined DESKTOP
				// use only local interfaces (127.0.0.1) or private address ranges, will also prevent to broadcast
				// outside a virtual or local network
				if (
				       (interfaceAddress > 0x0A000000 && interfaceAddress < 0x0AFFFFFF)  // 10.x.x.x
				    || (interfaceAddress > 0xAC0F0000 && interfaceAddress < 0xAC1FFFFF)  // 172.16.x.x through 172.31.x.x
				    || (interfaceAddress > 0xC0A80000 && interfaceAddress < 0xC0A8FFFF)  // 192.168.x.x
				    || (interfaceAddress > 0x7F000000 && interfaceAddress < 0x7F0000FF)  // 127.0.0.x
				   )
				{
#else
				// use all interfaces except the local one (we do not want to send to ourselves!)
				if (interfaceAddress > 0 && interfaceAddress != 0x7F000001) {
#endif
					struct sockaddr_in recipient = { 0 };
					recipient.sin_family      = AF_INET;
					recipient.sin_port        = htons(port);
					recipient.sin_addr.s_addr = broadcastAddress;

					sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient, sizeof recipient);
				}
			}
			p = p->ifa_next;
		}
		freeifaddrs(ifap);
	}
}
