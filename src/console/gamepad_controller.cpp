#include <robot_controller/gamepad.h>
#include <iostream>
#include <robot.h>
#include <easy_tcp/connection.h>

using namespace std;
using namespace robot;
using namespace easy_tcp;
using namespace cell_world::robot_controller;

int main(int argc, char *argv[]) {
    auto server = Connection::connect_remote("192.168.137.1", 4000);
    string message = "{\"command\":\"update_puff\",\"content\":\"\"}";
    //Robot robot("192.168.137.155",80);
    string device("/dev/input/js0");
    Gamepad j(device);
    bool h = true;
    bool enabled = true;
    int pleft = 0;
    int pright = 0;
    bool update = false;
    while (h){
        update = false;
        int left = -j.axes[1] * 3 /  256 / 4;
        int right = -j.axes[4] * 3 / 256 / 4;

//        cout << j.axes[0] << "\t" << j.axes[1] << "\t" << j.axes[2] << "\t" << j.axes[3] << "\t" << j.axes[4] << "\t" << j.axes[5] << "\t";
//	    for (int i = 0;i<j.buttons.size();i++) {
//            cout << j.buttons[i].state << "\t";
//        }
//	    cout << j.buttons[5].state << "\t" << (int) left << "\t" << (int) right << endl;


        if (j.buttons[8].state == 1) h = false;

//        if (pleft != left || pright != right) {
//            robot.set_left(left);
//            robot.set_right(right);
//            update = true;
//        }
        pleft = left;
        pright = right;
        if (j.buttons[5].state == 1){
            if (enabled) {
//                robot.set_puf();
                server.send_data(message.c_str(), message.size()+1);
                update = true;
            }
            enabled = false;
        } else {
            enabled = true;
        }
        if (update) {
//            robot.update();
        }
        usleep(30000);
    }
    return 0;
}
