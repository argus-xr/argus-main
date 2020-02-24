#include <ctime>
#include <string>
#include "tcp_server.h"

Server* server;

int main()
{
    server = new Server();
    server->start();
    while (true) {
        server->poll();
    }

    return 0;
}