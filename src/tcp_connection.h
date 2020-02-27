#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H

#include "kissnet.hpp"
#include "tcp_server.h"
#include "argus-netbuffer/netbuffer.h"
#include "argus-netbuffer/BasicMessageProtocol/BasicMessageProtocol.h"

class Server; // forward declaration

class Connection {
public:
	Connection(kissnet::tcp_socket&& socket, Server* owningServer);

	bool poll();
protected:
	kissnet::tcp_socket _socket;
	std::byte recvbuffer[1500];
	Server* owner = nullptr;
	BasicMessageBuffer* buf = new BasicMessageBuffer();


	void checkMessages();
	void processMessage(NetMessageIn* msg);
	void sendMessageRaw(const std::byte* buffer, size_t length);
};

#endif // TCP_CONNECTION_H