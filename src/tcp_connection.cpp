#include "tcp_connection.h"
namespace kn = kissnet;

#include "tcp_server.h"
#include <iostream>

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
	/*bool chop = false;
	do {
		chop = false;
		uint32_t chopSpot = 0;
		if (buf->getLength() <= 0) {
			return; // nothing to check.
		}
		//int32_t pos = buf->findByteSequence(delimeterSequence, delimeterSequence, 0);
		if (buf->getByteAt(0) == '|') {
			uint32_t len = buf->getInt();
			if (buf->size() > len) {
				std::uint8_t* msgBuf = new std::uint8_t[len - 4];
				buf->getBytes(msgBuf, len - 4);
				bb::ByteBuffer* msgBufBB = new bb::ByteBuffer(msgBuf, len - 4);
				processMessage(msgBufBB);
				chop = true;
				chopSpot = len + 5; // '|' and 4 bytes for length
			}
		}
		else {
			chopSpot = buf->find('|');
			if (chopSpot > 0) {
				chop = true;
			}
		}
		if (chop) { // remove a message and put the remainder in a new buffer.
			uint32_t length = buf->size() - chopSpot;
			std::uint8_t* tmpbuf = new std::uint8_t[length];
			buf->getBytes(tmpbuf, length);
			bb::ByteBuffer* tmp = buf;
			buf = new bb::ByteBuffer(tmpbuf, length);
			delete tmp;
		}
	} while (chop == true); // re-check until there's nothing to process.*/
}

void Connection::processMessage(NetMessageIn* msg) {
	uint8_t type = msg->readuint8();
	switch (type) {
	case 0:
		std::string s = msg->readFixedString(5);
		std::cout << s << "\n";
		break;
	}
	delete msg;
}