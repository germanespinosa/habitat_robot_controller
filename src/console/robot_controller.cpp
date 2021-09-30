#include <iostream>
#include <robot_controller.h>
#include <easy_tcp.h>

using namespace std;
using namespace json_cpp;
using namespace cell_world;
using namespace cell_world::robot_controller;
using namespace easy_tcp;

int main(int argc, char *argv[])
{
    int port = Robot_controller_service::port();

    // start server on port 65123
    Server<Robot_controller_service> server ;
    if (server.start(port)) {
        std::cout << "Server setup succeeded on port " << port << std::endl;
    } else {
        std::cout << "Server setup failed " << std::endl;
        return EXIT_FAILURE;
    }
    while(1);
    return 0;
}
