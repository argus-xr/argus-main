#ifndef CONNECTIONTCP_H
#define CONNECTIONTCP_H

#include "kissnet.hpp"
#include "ServerTCP.h"
#include "argus-netbuffer/netbuffer.h"
#include "argus-netbuffer/BasicMessageProtocol/BasicMessageProtocol.h"

class ServerTCP; // forward declaration

class ConnectionTCP {
public:
	ConnectionTCP(kissnet::tcp_socket&& socket, ServerTCP* owningServer);

	bool poll();
protected:
	kissnet::tcp_socket _socket;
	std::byte recvbuffer[1500];
	ServerTCP* owner = nullptr;
	BasicMessageBuffer* buf = new BasicMessageBuffer();


	void checkMessages();
	void processMessage(NetMessageIn* msg);
	void sendMessageRaw(const std::byte* buffer, size_t length);
	void processImageDataMessage(NetMessageIn* msg);
};

#endif // CONNECTIONTCP_H