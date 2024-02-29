/*
 * client.cpp
 *
 * A simple client that either sends strings to a server
 * or reads strings from a sever.
 *
 * Author: Jonathan Diller
 *
 */

#include <iostream>
#include <fstream>
#include <vector>

#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <netdb.h>
#include <stdio.h>

// Debug flag
#define DEBUG 0
// Message size limit
#define MAX_MSG_SIZE 3000
// (S)end (R)ecieve message size
#define S_R_SIZE 1

// Prints debugging message dbgString if DEBUG flag is set
void debugPrint(const char *dbgString) {
	if(DEBUG) {
		puts(dbgString);
	}
}

int main(int arg, char const *argv[]) {
	int nSock, nBytesRead, nBytesToSend = 0;
	char sBuffer[MAX_MSG_SIZE];
	struct addrinfo tConfigAddr, *tAddrSet, *tAddrInfo;
	const char cR[] = "R", cS[] = "S";

	// Verify arguments
	if(((strcmp(argv[3], cR) == 0) && (arg != 4)) ||
			((strcmp(argv[3], cS) == 0) && (arg != 5)) ||
			((strcmp(argv[3], cR) != 0) && (strcmp(argv[3], cS) != 0))) {
		fprintf(stderr,"ERROR: enter arguments: <hostname> <PORT> <S or R> [file]\n");
		exit(1);
	}

	// If sending a message, read text file
	if(strcmp(argv[3], cS) == 0) {
		debugPrint("Reading file");

		std::ifstream file(argv[4]);
		std::string sInptStr;

		int j = 0;
		while((std::getline(file, sInptStr)) && (j < MAX_MSG_SIZE)) {
			debugPrint("Read line from file");

			// Add file text to buffer
			for(int i = 0;(i < sInptStr.length()) && (j < MAX_MSG_SIZE); i++, j++) {
				sBuffer[j] = sInptStr[i];
			}
			sBuffer[j++] = '\n';

		}

		nBytesToSend = j;

		// Add terminator, remove last return, close file
		sBuffer[j - 1] = '\0';
		file.close();

		debugPrint(sBuffer);
		debugPrint("Done reading file");
	}

	// Configure the socket type that we want
	memset(&tConfigAddr, 0, sizeof tConfigAddr);
	tConfigAddr.ai_family = AF_UNSPEC;
	tConfigAddr.ai_socktype = SOCK_STREAM; // TCP

	// Get a set of socket addresses
	int nRVal = getaddrinfo(argv[1], argv[2], &tConfigAddr, &tAddrSet);
	if(nRVal != 0) {
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
		fprintf(stderr,"ERROR: failed to connect\n");
		exit(1);
	}

	// Free list
	freeaddrinfo(tAddrSet);

	// Handle (S)end or (R)eceive command
	if(strcmp(argv[3], cS) == 0) {
		// Tell server we want send a message
		send(nSock , cS, strlen(cS), 0);
		debugPrint("'Send' message sent");

		char sSRCmd[S_R_SIZE + 1];

		// Read server's response
		nBytesRead = read(nSock, sSRCmd, S_R_SIZE);
		sSRCmd[S_R_SIZE] = '\0';

		if(strcmp(sSRCmd, cR) == 0) {
			// Server is ready to receive message!
			// Send message
			send(nSock , sBuffer, nBytesToSend, 0);
			debugPrint("Message sent");
		}
		else {
			fprintf(stderr,"ERROR: server did not send 'receive' message\nReceived \"%s\"\n", sSRCmd);
			exit(1);
		}

	}
	else if(strcmp(argv[3], cR) == 0) {
		// Tell server we want to receive a message
		send(nSock , cR, strlen(cR) , 0 );
		debugPrint("'Receive' message sent");

		// Read from server
		nBytesRead = read(nSock, sBuffer, MAX_MSG_SIZE-1);

		// Verify data was read
		if(nBytesRead == -1) {
			fprintf(stderr,"ERROR: did not receive data socket\n");
			exit(1);
		}

		// Print data read
		sBuffer[nBytesRead] ='\0';
		printf("Message from server: \n%s\n", sBuffer);
	}

	close(nSock);

	return 0;
}


