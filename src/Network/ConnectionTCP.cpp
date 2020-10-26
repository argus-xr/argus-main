#include "ConnectionTCP.h"
namespace kn = kissnet;

#include "ServerTCP.h"
#include <iostream>
#include "Events.h"
#include "ControllerEvents.h"
#include "RandomWrapper.h"
#include "ArgusControllerManager.h"
#include "ArgusConfig.h"

#ifdef SDL_FOUND
#include "SDLUI.h"
#endif

ConnectionTCP::ConnectionTCP(kissnet::tcp_socket&& socket, ServerTCP* owningServer)
{
	srand((unsigned)time(0));
	_socket = std::move(socket);
	owner = owningServer;

	sendHandshakeMessage();
}

bool ConnectionTCP::poll() {
	bool continue_receiving = true;

	do {
		auto [size, valid] = _socket.recv(recvbuffer, 1500);
		if (valid && size > 0) {
			buf->insertBuffer((uint8_t*) recvbuffer, size, true);
			checkMessages();
			printf("Received %llu bytes.\n", size);
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
	CTSMessageType type = (CTSMessageType) msg->readVarInt();
	switch (type) {
	case CTSMessageType::Handshake:
		processHandshakeMessage(msg);
		break;
	case CTSMessageType::GUIDResponse:
		processGUIDResponseMessage(msg);
		break;
	case CTSMessageType::VideoData:
		processImageDataMessage(msg);
		break;
	case CTSMessageType::IMUData:
		processIMUDataMessage(msg);
		break;
	case CTSMessageType::Debug:
		processDebugMessage(msg);
		break;
	default:
		break;
	}
	delete msg;
}

void ConnectionTCP::sendHandshakeMessage() {
	NetMessageOut* msg = new NetMessageOut(0);
	msg->writeVarInt((uint64_t)STCMessageType::Handshake);
	std::string contents = msg->debugBuffer();
	printf("Sending handshake, contents: %s.\n", contents.c_str());
	sendMessage(msg);
}

void ConnectionTCP::processHandshakeMessage(NetMessageIn* msg) {
	uint64_t guid = msg->readVarInt(); // no uint64_t support yet
	std::string contents = msg->debugBuffer();
	printf("Processing handshake - GUID: %llu, contents: %s.\n", guid, contents.c_str());
	// if guid = 0, create random guid. If guid does not match a known controller, add a new controller entry. Assign controller config to connection.
	if (guid == 0) {
		guid = RandomWrapper::getRandomUInt64();
		sendSetGUIDMessage(guid);
	}
	std::shared_ptr<ArgusController> cnt = ArgusControllerManager::getController(guid);
	if (cnt) {
		controller = cnt;
	}
	else {
		std::shared_ptr<ControllerConfig> cfg = ArgusConfig::getControllerConfig(guid);
		if (!cfg) {
			cfg = std::shared_ptr<ControllerConfig>(new ControllerConfig());
			cfg->guid = guid;
			ArgusConfig::setControllerConfig(cfg);
		}
		controller = std::shared_ptr<ArgusController>(new ArgusController(cfg));
	}
}

void ConnectionTCP::sendSetGUIDMessage(uint64_t newGUID) {
	NetMessageOut* msg = new NetMessageOut(0);
	msg->writeVarInt((uint64_t)STCMessageType::SetGUID);
	msg->writeVarInt(newGUID);
	std::string contents = msg->debugBuffer();
	printf("Sending SetGUID - GUID: %llu, contents: %s.\n", newGUID, contents.c_str());
	sendMessage(msg);
}

void ConnectionTCP::processGUIDResponseMessage(NetMessageIn* msg) {
	// guid has been accepted, presumably, so continue.
	printf("New GUID accepted.\n");
}

void ConnectionTCP::processImageDataMessage(NetMessageIn* msg) {
	uint64_t timestamp = msg->readVarInt();
	uint64_t bufLen = msg->readVarInt();
	uint8_t* buf = msg->readByteBlob((uint32_t)bufLen);
#ifdef SDL_FOUND
	auto frame = std::shared_ptr<VideoFrame>(new VideoFrame(buf, bufLen, VideoFrame::VideoFrameEncoding::VFE_JPEG));
	ArgusVizUI::inst()->setNewFrame(frame);
	if (controller) {
		controller->setVideoFrame(frame, timestamp);
	}
	printf("New videoframe received. Timestamp: %llu.\n", timestamp);
#endif
}

void ConnectionTCP::processIMUDataMessage(NetMessageIn* msg) {
	uint64_t num = msg->readVarInt();
	IMUData data;
	const float AMult = 8.0f * 9.81f / INT16_MAX; // accelerometer multiplier, converting to m/s^2
	const float GMult = 1000.0f / INT16_MAX; // gyro multiplier, converting to deg/sec
	for (int i = 0; i < num; ++i) {
		data.aX = (float)msg->readVarIntSigned() * AMult;
		data.aY = (float)msg->readVarIntSigned() * AMult;
		data.aZ = (float)msg->readVarIntSigned() * AMult;
		data.gX = (float)msg->readVarIntSigned() * GMult;
		data.gY = (float)msg->readVarIntSigned() * GMult;
		data.gZ = (float)msg->readVarIntSigned() * GMult;
		data.timestamp_us = msg->readVarInt(); // timestamp in microseconds.
		/*if (controller) {
			controller->addIMUData(data);
			controller->printTracking();
		}*/
		//else {
			//printf("IMU: %6.2f %6.2f %6.2f - %6.2f %6.2f %6.2f, timestamp %llu.\n", data.aX, data.aY, data.aZ, data.gX, data.gY, data.gZ, data.timestamp_us);
		//}
	}
}

void ConnectionTCP::processDebugMessage(NetMessageIn* msg) {
	std::string s = msg->readVarString();
	std::cout << s << "\n";
}

void ConnectionTCP::sendMessageRaw(const std::byte* buffer, size_t length) {
	_socket.send(buffer, length);
}

void ConnectionTCP::sendMessage(NetMessageOut* msg) {
	OutPacket* pac = buf->messageToOutPacket(msg);
	sendMessageRaw((std::byte*) pac->getData(), pac->getDataLength());
	delete pac;
	delete msg;
}