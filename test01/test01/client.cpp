#include <stdio.h>
#include <WinSock2.h>

#pragma comment(lib,"Ws2_32.lib")

void clientMain() {
	WSADATA wsa;
	SOCKET s;

	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		printf("Failed. Error Code : %d", WSAGetLastError());
	}
	printf("Initialised.");

	if ((s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)) == INVALID_SOCKET) {
		printf("Could not create socet : %d", WSAGetLastError());
	}
	printf("Socket created.\n");
}