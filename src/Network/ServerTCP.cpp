#include <iostream>

#include "ServerTCP.h"
namespace kn = kissnet;

#include "ConnectionTCP.h"

void ServerTCP::start() {
}

void ServerTCP::poll() {
    if (!listening) {
        try {
            listen_socket = std::move(kn::tcp_socket({ "0.0.0.0", port }));

            listen_socket.set_non_blocking(true);
            listen_socket.bind();
            listen_socket.listen();
            listening = true;
            std::cout << "Listening on port " << port << ".\n";
        }
        catch (std::exception e) {
            if (!failingToListen) {
                failingToListen = true;
                std::cout << "Failing to listen on port " << port << ", retrying.\n";
            }
        }
    }
    if (listening) {
        try {
            kn::tcp_socket socket = listen_socket.accept();
            if (socket != NULL && socket.is_valid()) {
                ConnectionTCP* conn = new ConnectionTCP(std::move(socket), this);
                connections.push_back(conn);
                std::cout << "New connection accepted.\n";
            }
        }
        catch (std::exception e) {

        }
        for (ConnectionTCP* c : connections) {
            c->poll();
        }
    }
}