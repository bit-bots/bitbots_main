#ifndef MITECOM_NETWORK_H__
#define MITECOM_NETWORK_H__

#include <stdio.h>
#include <inttypes.h>

int mitecom_open(int port);
ssize_t mitecom_receive(int sock, void* buffer, unsigned int bufferSize);
void mitecom_broadcast(int socket, int port, const void* data, uint32_t dataLength);


#endif
