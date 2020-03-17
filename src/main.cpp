#include <ctime>
#include <string>
#include "ServerTCP.h"

#ifdef SDL_FOUND
#include "SDLUI.h"
#include <SDL.h>
#undef main // SDL.h has #define main SDL_main which causes issues.
#endif // SDL_FOUND

ServerTCP* server;

int main(int argc, char* argv[]) {
#ifdef SDL_FOUND
    ArgusVizUI* ui = new ArgusVizUI();
    ui->start();
#endif
    server = new ServerTCP();
    server->start();
    while (true) {
        server->poll();
#ifdef SDL_FOUND
        ui->poll();
#endif
    }
    return 0;
}