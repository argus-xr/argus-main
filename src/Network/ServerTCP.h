#ifndef SERVERTCP_H
#define SERVERTCP_H

#include <vector>
#include "kissnet.hpp"

class ConnectionTCP; // forward declaration

class ServerTCP {
public:
    void start();
    void poll();

protected:
    kissnet::port_t port = 11000;
    kissnet::tcp_socket listen_socket; // invalid socket until reassigned later
    bool listening = false;
    bool failingToListen = false;
    std::vector<ConnectionTCP*> connections;
};

#endif // SERVERTCP_H