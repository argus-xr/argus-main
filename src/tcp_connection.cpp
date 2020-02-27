#include "tcp_connection.h"
namespace kn = kissnet;

#include "tcp_server.h"
#include <iostream>

Connection::Connection(kissnet::tcp_socket&& socket, Server* owningServer)
{
	_socket = std::move(socket);
	owner = owningServer;

	NetMessageOut* msg = new NetMessageOut(5);
	msg->writeuint8(0);
	msg->writeVarString("Test");
	uint8_t* msgBuf;
	uint32_t length = buf->messageOutToByteArray(msgBuf, msg);
	sendMessageRaw((std::byte*) msgBuf, length);
	delete[] msgBuf;
	delete msg;
}

bool Connection::poll() {
	bool continue_receiving = true;

	do {
		auto [size, valid] = _socket.recv(recvbuffer, 1500);
		if (valid && size > 0) {
			buf->insertBuffer((uint8_t*) recvbuffer, size, true);
			checkMessages();
		}
		else {
			continue_receiving = false;
		}
	} while (continue_receiving);
	return true;
}

void Connection::checkMessages() {
	buf->checkMessages();
	NetMessageIn* msg;

	while(true) {
		msg = buf->popMessage();
		if (msg == nullptr) {
			break;
		}
		else {
			processMessage(msg);
		}
	}
}

void Connection::processMessage(NetMessageIn* msg) {
	uint8_t type = msg->readuint8();
	switch (type) {
	case 0:
		std::string s = msg->readVarString();
		std::cout << s << "\n";
		break;
	}
	delete msg;
}

void Connection::sendMessageRaw(const std::byte* buffer, size_t length) {
	_socket.send(buffer, length);
}