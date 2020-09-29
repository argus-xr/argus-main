#ifndef CONNECTIONTCP_H
#define CONNECTIONTCP_H

#include "kissnet.hpp"
#include "ServerTCP.h"
#include "argus-netbuffer/netbuffer.h"
#include "argus-netbuffer/BasicMessageProtocol/BasicMessageProtocol.h"

class ServerTCP; // forward declaration

enum class CTSMessageType { // client to server
    Handshake,
    GUIDResponse,
    VideoData = 20,
    IMUData,
    Debug = 10000,
};

enum class STCMessageType { // server to client
    Handshake,
    SetGUID,
    Debug = 10000,
};

class ConnectionTCP {
public:
	ConnectionTCP(kissnet::tcp_socket&& socket, ServerTCP* owningServer);

	bool poll();
protected:
	uint64_t guid = 0;
	kissnet::tcp_socket _socket;
	std::byte recvbuffer[1500];
	ServerTCP* owner = nullptr;
	BasicMessageBuffer* buf = new BasicMessageBuffer();


	void checkMessages();
	void processMessage(NetMessageIn* msg);
	void sendMessageRaw(const std::byte* buffer, size_t length);
	void sendMessage(NetMessageOut* msg);

	void sendHandshakeMessage();
	void processHandshakeMessage(NetMessageIn* msg);
	void sendSetGUIDMessage(uint64_t newGUID);
	void processGUIDResponseMessage(NetMessageIn* msg);

	void processImageDataMessage(NetMessageIn* msg);
	void processIMUDataMessage(NetMessageIn* msg);
	void processDebugMessage(NetMessageIn* msg);
};

#endif // CONNECTIONTCP_H