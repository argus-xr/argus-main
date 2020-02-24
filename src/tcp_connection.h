#ifndef TCP_CONNECTION_H
#define TCP_CONNECTION_H

#include "kissnet.hpp"
#include "tcp_server.h"
//#include "ByteBuffer.hpp"
#include "netbuffer.h"
#include "BasicMessageProtocol/BasicMessageProtocol.h"

class Server; // forward declaration

class Connection {
public:
	Connection(kissnet::tcp_socket&& socket, Server* owningServer) {
		_socket = std::move(socket);
		owner = owningServer;
	}

	bool poll();
protected:
	kissnet::tcp_socket _socket;
	std::byte recvbuffer[1500];
	Server* owner = nullptr;
	//bb::ByteBuffer* buf = new bb::ByteBuffer();
	BasicMessageBuffer* buf = new BasicMessageBuffer();

	const uint8_t* delimeterSequence = new uint8_t[2]{'\\', '\0'};
	const uint32_t delimterLength = 2;


	void checkMessages();
	void processMessage(NetMessageIn* msg);
};

#endif // TCP_CONNECTION_H