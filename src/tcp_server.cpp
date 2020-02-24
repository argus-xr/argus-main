#include <iostream>

#include "tcp_server.h"
namespace kn = kissnet;

#include "tcp_connection.h"

void Server::start() {
    listen_socket = std::move(kn::tcp_socket({ "0.0.0.0", port }));
    
    listen_socket.set_non_blocking(true);
    listen_socket.bind();
    listen_socket.listen();
}

void Server::poll() {
    kn::tcp_socket socket = listen_socket.accept();
    if (socket != NULL && socket.is_valid()) {
        Connection* conn = new Connection(std::move(socket), this);
        connections.push_back(conn);
        std::cout << "Connected!\n";
    }
    for (Connection* c : connections) {
        c->poll();
    }
}