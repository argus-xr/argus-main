#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <vector>
#include "kissnet.hpp"

class Connection; // forward declaration

class Server {
public:
    void start();
    void poll();

protected:
    kissnet::port_t port = 11000;
    kissnet::tcp_socket listen_socket; // invalid socket until reassigned later
    std::vector<Connection*> connections;
};

#endif // TCP_SERVER_H