
#include <sstream>
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
 ** @return -1 on error, otherwise the socket descriptor
 */
 static bool mitecom_sim_enabled = getenv("MITECOM") != NULL;


int mitecom_open(int port) {
	int sock;

	// create socket
	if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
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
 **
 */

ssize_t mitecom_receive(int sock, void* buffer, int bufferSize) {
	struct sockaddr_in remoteAddress;
	socklen_t addressLen = sizeof(remoteAddress);
	return recvfrom(sock, buffer, bufferSize, 0, (struct sockaddr *)&remoteAddress, &addressLen);
}


/*------------------------------------------------------------------------------------------------*/

/** Send a broadcast message from the given socket to a port.
 **
 ** @param socket   The outgoing (local) socket
 ** @param port     The port to broadcast to
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


                    // Fake the broadcasts between machines by sending the data to different ports
                    // Simulated clients will listen on different ports on the same machine
                    if (mitecom_sim_enabled){
                        struct sockaddr_in recipient1 = { 0 };
                        recipient1.sin_family      = AF_INET;
                        recipient1.sin_port        = htons(12121);
                        recipient1.sin_addr.s_addr = broadcastAddress;
                        sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient1, sizeof recipient1);

                        struct sockaddr_in recipient2 = { 0 };
                        recipient2.sin_family      = AF_INET;
                        recipient2.sin_port        = htons(12221);
                        recipient2.sin_addr.s_addr = broadcastAddress;
                        sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient2, sizeof recipient2);

                        struct sockaddr_in recipient3 = { 0 };
                        recipient3.sin_family      = AF_INET;
                        recipient3.sin_port        = htons(12321);
                        recipient3.sin_addr.s_addr = broadcastAddress;
                        sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient3, sizeof recipient3);

                        struct sockaddr_in recipient4 = { 0 };
                        recipient4.sin_family      = AF_INET;
                        recipient4.sin_port        = htons(12421);
                        recipient4.sin_addr.s_addr = broadcastAddress;
                        sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient4, sizeof recipient4);

                        struct sockaddr_in recipient5 = { 0 };
                        recipient5.sin_family      = AF_INET;
                        recipient5.sin_port        = htons(12521);
                        recipient5.sin_addr.s_addr = broadcastAddress;
                        sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient5, sizeof recipient5);
                    }else{
                        // Normal CAse
                        struct sockaddr_in recipient = { 0 };
                        recipient.sin_family      = AF_INET;
                        recipient.sin_port        = htons(port);
                        recipient.sin_addr.s_addr = broadcastAddress;

                        sendto(sock, data, dataLength, 0, (const struct sockaddr*)&recipient, sizeof recipient);
                    }



				}
			}
			p = p->ifa_next;
		}
		freeifaddrs(ifap);
	}
}
