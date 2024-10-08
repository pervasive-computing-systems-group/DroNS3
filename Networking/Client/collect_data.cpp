/*
 * collect_data.cpp
 *
 * A simple client-type application that attempts to connect to a 
 * specific ground node (server-type) to collect data from it
 *
 * Author: Jonathan Diller
 * Data: October 13, 2022
 *
 */

#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <stdio.h>

#include "defines.h"

// hostname and port
char const *info[] = {"localhost", "8080"};


int main(int argc, char** argv) {
	const char* hostname;

	if(argc != 2) {
		printf("ERROR: expected argument for node hostname, e.g.:\n ./collect_data localhost\n");
		return 1;
	}

	// Get node id from arguments
	hostname = argv[1];
	
	if(DEBUG)
		printf("Collecting data from node at %s\n", hostname);

	int nSock, nBytesRead;
	struct addrinfo tConfigAddr, *tAddrSet, *tAddrInfo;

	// Configure the socket type that we want
	memset(&tConfigAddr, 0, sizeof tConfigAddr);
	tConfigAddr.ai_family = AF_UNSPEC;
	tConfigAddr.ai_socktype = SOCK_STREAM; // TCP

	// Get a set of socket addresses
	int nRVal = getaddrinfo(hostname, DATA_PORT, &tConfigAddr, &tAddrSet);
	if(nRVal != 0) {
		if(DEBUG)
			fprintf(stderr,"ERROR: getaddrinfo() failed: %s\n", gai_strerror(nRVal));
		exit(1);
	}

	tAddrInfo = tAddrSet;

	// Loop through addresses and try to connect
	while(tAddrInfo != NULL) {
		// Create socket
		nSock = socket(tAddrInfo->ai_family, tAddrInfo->ai_socktype, tAddrInfo->ai_protocol);
		if(nSock == -1) {
			debugPrint("Trying to connect to socket");
			tAddrInfo = tAddrInfo->ai_next;

			continue;
		}

		// Attempt to connect to server socket
		if(connect(nSock, tAddrInfo->ai_addr, tAddrInfo->ai_addrlen) == -1) {
			// Failed to connect
			close(nSock);
			debugPrint("Failed to connect to socket");
			tAddrInfo = tAddrInfo->ai_next;

			continue;
		}
		else {
			break;
		}
	}

	if(tAddrInfo == NULL) {
		if(DEBUG)
			fprintf(stderr,"ERROR: failed to connect\n");
		exit(1);
	}

	// Free list
	freeaddrinfo(tAddrSet);

	// Create a packet to send to server node
	packet_t sendPack;
	sendPack.id = 7;
	sendPack.msg_type = REQUEST;

	// Send message
	send(nSock , &sendPack, sizeof(packet_t), 0);
	debugPrint("Message sent");

	packet_t recPack;

	// Read from server node
	nBytesRead = read(nSock, &recPack, sizeof(packet_t));

	// Verify data was read
	if(nBytesRead == -1) {
		if(DEBUG)
			fprintf(stderr,"ERROR: did not receive data socket\n");
		exit(1);
	}
	else {
		// We successfully received a client message
		printf("Received response\n packet ID: %d, type: %d\nSize of data to collect: %d\n", recPack.id, recPack.msg_type, recPack.bytes_to_send);

		// Create a data structure to hold the data
		char* data = new char[recPack.bytes_to_send];

		// Send the rest of the data...
		int bytes_read = 0;
		while(bytes_read < recPack.bytes_to_send) {
			bytes_read += read(nSock, data + bytes_read, recPack.bytes_to_send - bytes_read);
			debugPrint(" read data packet");
		}
		printf("Read %d bytes\n", bytes_read);
		debugPrint(data);

		// Memory cleanup
		delete[] data;
	}

	close(nSock);

	debugPrint("Successfully collected data from server\n");

//	// We have successfully contacted the node. Sleep to simulate data collection
//	std::this_thread::sleep_for(std::chrono::milliseconds(collectTime));

	return 0;
}
