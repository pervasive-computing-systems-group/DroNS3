/*
 * server.cpp
 *
 * A server application that receives messages from clients and responds
 * with a simple acknowledgment message
 *
 * Author: Jonathan Diller
 * Data: October 13, 2022
 *
 */

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <stdio.h>

#include "defines.h"

// Number of pending connections to queue
#define CONNECTION_QUEUE_SIZE 10

#define DEFAULT_BYTE_TO_SEND 1000000

int main(int arg, char const *argv[]) {
	int nListeningSock, nRVal, nReuse = 1;

	struct addrinfo tConfigAddr, *tAddrSet, *tAddrInfo;

	// Configure the server socket type
	memset(&tConfigAddr, 0, sizeof tConfigAddr);
	tConfigAddr.ai_family = AF_UNSPEC;
	tConfigAddr.ai_socktype = SOCK_STREAM; // TCP
	tConfigAddr.ai_flags = AI_PASSIVE; // Use local machine IP

	// Get a set of socket addresses
	nRVal = getaddrinfo(NULL, DATA_PORT, &tConfigAddr, &tAddrSet);
	if(nRVal) {
		fprintf(stderr,"ERROR: getaddrinfo() failed: %s\n", gai_strerror(nRVal));
		exit(1);
	}

	tAddrInfo = tAddrSet;

	// Loop through addresses and try to bind to a socket
	while(tAddrInfo != NULL) {
		// Create listening socket
		nListeningSock = socket(tAddrInfo->ai_family, tAddrInfo->ai_socktype, tAddrInfo->ai_protocol);
		if(nListeningSock == -1) {
			debugPrint("Trying to connect to socket");
			tAddrInfo = tAddrInfo->ai_next;

			continue;
		}

		// Set socket option to reuse address
		if(setsockopt(nListeningSock, SOL_SOCKET, SO_REUSEADDR, &nReuse, sizeof(int)) == -1) {
			fprintf(stderr,"ERROR: setsockopt() failed\n");
			exit(1);
		}

		// Attempt to bind address info to socket
		if(bind(nListeningSock, tAddrInfo->ai_addr, tAddrInfo->ai_addrlen) == -1) {
			close(nListeningSock);
			debugPrint("Failed to bind socket");
			tAddrInfo = tAddrInfo->ai_next;
			continue;
		}
		else {
			break;
		}
	}

	if(tAddrInfo == NULL) {
		fprintf(stderr,"ERROR: failed to bind socket\n");
		exit(1);
	}

	// Free list
	freeaddrinfo(tAddrSet);

	// Enable socket to accept incoming connections, with client queue
	if(listen(nListeningSock, CONNECTION_QUEUE_SIZE) == -1) {
		fprintf(stderr,"ERROR: unable to set socket to listen\n");
		exit(1);
	}

	// How much data are we sending?
	int byte_to_send = DEFAULT_BYTE_TO_SEND;
	if(arg == 2) {
		byte_to_send = atoi(argv[1]);
	}

	// Create bytes
	char* data = new char[byte_to_send];

	// Fill with garbage (for sanity check at receiver)
	for(int i = 0; i < byte_to_send; i++) {
		data[i] = char((i%26)+ 65);
	}

	printf("Server is ready!\n");


	int nClientSocket;
	struct sockaddr_storage tClientAddr;
	socklen_t tpClientAddrSize;

	// Infinite loop to handle incoming connections
	while(true) {
		tpClientAddrSize = sizeof(tClientAddr);

		// Wait for incoming connection requests
		nClientSocket = accept(nListeningSock, (struct sockaddr *)&tClientAddr, &tpClientAddrSize);

		// Verify we connected
		if(nClientSocket == -1) {
			debugPrint("Failed to accept connection request");
			continue;
		}

		packet_t recPack;

		// Read from server
		int nBytesRead = read(nClientSocket, &recPack, sizeof(packet_t));

		// Verify data was read
		if(nBytesRead == -1) {
			// Failed to read, close socket and end child process
			fprintf(stderr,"ERROR: did not read client hello message\n");

			close(nClientSocket);
			continue;
		}
		else {
			// We successfully received a client message
			printf("Received request\n packet ID: %d, type: %d\n", recPack.id, recPack.msg_type);

			// Send acknowledgment packet back to client
			packet_t sendPack;
			sendPack.id = (recPack.id  + 1);
			sendPack.msg_type = RESPONSE;
			sendPack.bytes_to_send = byte_to_send;

			// Send message
			if(send(nClientSocket , &sendPack, sizeof(packet_t), 0) == -1) {
				fprintf(stderr,"ERROR: failed to send acknowledgment message to client\n");
			}
			else {
				debugPrint("Message acknowledgment\nSending Data");

				// Send the rest of the data...
				int bytes_sent = 0;
				while(bytes_sent < byte_to_send) {
					bytes_sent += send(nClientSocket , data+bytes_sent, byte_to_send - bytes_sent, 0);
					debugPrint(" sent data packet");
				}
				printf("Sent %d bytes\n", bytes_sent);
			}
		}

		debugPrint("Client handled!");
		close(nClientSocket);
	}

	return 0;
}


