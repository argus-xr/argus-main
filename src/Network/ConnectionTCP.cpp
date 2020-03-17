#include "ConnectionTCP.h"
namespace kn = kissnet;

#include "ServerTCP.h"
#include <iostream>
#include "Events.h"
#include "ControllerEvents.h"
#include "SDLUI.h"

#ifdef SDL_FOUND
#include "SDLUI.h"
#endif

ConnectionTCP::ConnectionTCP(kissnet::tcp_socket&& socket, Server* owningServer)
{
	_socket = std::move(socket);
	owner = owningServer;

	/*NetMessageOut* msg = new NetMessageOut(5);
	msg->writeuint8(0);
	msg->writeVarString("Test\\Moretesting");
	uint8_t* msgBuf;
	uint32_t length = buf->messageOutToByteArray(msgBuf, msg);
	sendMessageRaw((std::byte*) msgBuf, length);
	delete[] msgBuf;
	delete msg;*/
}

bool ConnectionTCP::poll() {
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

void ConnectionTCP::checkMessages() {
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

void ConnectionTCP::processMessage(NetMessageIn* msg) {
	uint64_t type = msg->readVarInt();
	switch (type) {
	case 0:
	{
		std::string s = msg->readVarString();
		std::cout << s << "\n";
		break;
	}
	case 1:
		processImageDataMessage(msg);
		break;
	}
	delete msg;
}

void ConnectionTCP::processImageDataMessage(NetMessageIn* msg) {
	uint64_t bufLen = msg->readVarInt();
	uint8_t* buf = msg->readByteBlob((uint32_t)bufLen);
	VideoFrame* frame = new VideoFrame(buf, bufLen, VideoFrame::VideoFrameEncoding::VFE_JPEG);
#ifdef SDL_FOUND
	ArgusVizUI::inst()->setNewFrame(std::shared_ptr<VideoFrame>(frame));
#endif
}

void ConnectionTCP::sendMessageRaw(const std::byte* buffer, size_t length) {
	_socket.send(buffer, length);
}