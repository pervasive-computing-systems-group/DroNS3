/*
 * server.cpp
 *
 * A server application that receives messages from clients,
 * stores messages in a queue, and returns messages when queried.
 *
 * Author: Jonathan Diller
 *
 */

#include <queue>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <stdio.h>

// Debug flag
#define DEBUG 0
// Number of pending connections to queue
#define CONNECTION_QUEUE_SIZE 10
// Message size limit
#define MAX_MSG_SIZE 3000
// (S)end (R)ecieve message size
#define S_R_SIZE 1

struct packet_t {
	packet_t(int buffSize, char buffer[MAX_MSG_SIZE]) {
		this->m_nBuffSize = buffSize;
		strcpy(this->m_sBuffer, buffer);
	}

	int m_nBuffSize;
	char m_sBuffer[MAX_MSG_SIZE];
};

// Prints debugging message dbgString if DEBUG flag is set
void debugPrint(const char *dbgString) {
	if(DEBUG) {
		puts(dbgString);
	}
}

int main(int arg, char const *argv[]) {
	int nListeningSock, nClientSocket, nRVal, nReuse = 1;
	const char cR[] = "R", cS[] = "S";
	char sBuffer[MAX_MSG_SIZE];

	struct addrinfo tConfigAddr, *tAddrSet, *tAddrInfo;
	struct sockaddr_storage tClientAddr;
	socklen_t tpClientAddrSize;

	std::queue<packet_t> qPacketQueue;

	// Configure the server socket type
	memset(&tConfigAddr, 0, sizeof tConfigAddr);
	tConfigAddr.ai_family = AF_UNSPEC;
	tConfigAddr.ai_socktype = SOCK_STREAM; // TCP
	tConfigAddr.ai_flags = AI_PASSIVE; // Use local machine IP

	// Verify arguments
	if(arg !=2) {
		fprintf(stderr,"ERROR: enter port\n");
		exit(1);
	}

	// Get a set of socket addresses
	nRVal = getaddrinfo(NULL, argv[1], &tConfigAddr, &tAddrSet);
	if(nRVal) {
		fprintf(stderr,"ERROR: getaddrinfo() failed: %s\n", gai_strerror(nRVal));
		exit(1);
	}

	tAddrInfo = tAddrSet;

	// Loop through addresses and try to connect
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

	printf("Server is ready!\n");

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

		char sSRCmd[S_R_SIZE + 1];

		// Read from server
		int nBytesRead = read(nClientSocket, sSRCmd, S_R_SIZE);
		sSRCmd[S_R_SIZE] ='\0';

		// Verify data was read
		if(nBytesRead == -1) {
			// Failed to read, close socket and end child process
			fprintf(stderr,"ERROR: did not read client hello message\n");

			close(nClientSocket);
		}
		int lastBytesRead = 0;

		// Handle read/write request
		if(strcmp(sSRCmd, cS) == 0) {
			// Client wants to send us a message
			// Tell client we are ready to receive
			if(send(nClientSocket , cR, strlen(cR), 0) == -1) {
				fprintf(stderr,"ERROR: failed to send 'receive' message to client\n");
			}

			// Read from server
			nBytesRead = read(nClientSocket, sBuffer, MAX_MSG_SIZE);

			//update the bytes that were read from most recent message from client
			lastBytesRead = nBytesRead;

			// Print data read
			sBuffer[nBytesRead] ='\0';
			printf("Message from client: \n%s\n",sBuffer);

			qPacketQueue.push(packet_t(nBytesRead, sBuffer));

			debugPrint("Added message to packet buffer");
		}
		else if(strcmp(sSRCmd, cR) == 0) {
			// Client wants to read data!
			// Send message from packet buffer (if there is one to send)
			if(!qPacketQueue.empty()) {
				packet_t packet = qPacketQueue.front();
				char byteCountMsg[100];
				sprintf(byteCountMsg, "Bytes read from last message: %d", lastBytesRead);
				if (write(nClientSocket, byteCountMsg, strlen(byteCountMsg)) == -1){
					fprintf(stderr, "ERROR: failed to write to client\n");
				}
				lastBytesRead = 0;
				// Send message to client
				//shoudln't need to send the packetQueue
				/**
				if(write(nClientSocket, packet.m_sBuffer, packet.m_nBuffSize) == -1) {
					fprintf(stderr,"ERROR: failed to write to client\n");
				}*/
				qPacketQueue.pop();
			}
			else {
				// Buffer is empty, tell to client
				if(write(nClientSocket, "Error, no messages!", 19) == -1) {
					fprintf(stderr,"ERROR: failed to write to client\n");
				}

				debugPrint("Buffer empty! :o");
			}
		}
		else {
			// Client sent unexpected hello message
			fprintf(stderr,"ERROR: receive unexpected hello message: \"%s\"\n", sSRCmd);
		}

		debugPrint("Client handled!");
		close(nClientSocket);
	}

	return 0;
}


