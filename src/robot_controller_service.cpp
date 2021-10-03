#include <robot.h>
#include <cell_world_tools.h>
#include <filesystem>
#include <math.h>
#include <robot_controller/robot_controller_service.h>


using namespace json_cpp;
using namespace std;
using namespace robot;
using namespace cell_world;

namespace cell_world::robot_controller {

    struct Occlusions : Json_object {
        Json_object_members(
                Add_member(OcclusionIds);
        );
        Json_vector<unsigned int> OcclusionIds;
    };

    struct Data{
        Data (const std::string &world_name):
                world(Json_create<World>(Web_resource::from("world").key(world_name).get())),
                cells(world.create_cell_group()),
                map(cells),
                visibility(world.create_graph(Json_create<Graph_builder>(Web_resource::from("graph").key(world_name).key("visibility").get()))),
                paths(world.create_paths(Json_create<Path_builder>(Web_resource::from("paths").key(world.name).key("astar").get()))){
        }
        World world;
        Cell_group cells;
        Map map;
        Graph visibility;
        Paths paths;
    };

    Data *data = new Data("hexa_10_00_vr");
    double speed = 100;
    double view_angle = 145 * 2 * M_PI / 360;
    unsigned int min_ghost_distance = 5;
    Cell_group spawn_locations;
    Robot robot("192.168.137.155",80);
    Agent_state prey;
    Agent_state predator;

    bool Robot_controller_service::update_spawn_locations (){
        auto start = data->map[{-20,0}];
        spawn_locations.clear();
        for (auto &cell : data->cells){
            if (!cell.get().occluded && data->paths.get_steps(start,cell) >= (int)min_ghost_distance) {
                spawn_locations.add(cell);
            }
        }
        cout << spawn_locations << endl;
        if (spawn_locations.empty()) return false;
        return true;
    }

    void Robot_controller_service::on_connect()  {
    }

    void Robot_controller_service::on_incoming_data(const std::string &plugin_data)  {
        Message request;
        Message response;
        try {
            plugin_data >> request;
        } catch (...) {
            return; // ignores malformed messages
        }
        if (request.command == "start_episode") {
            //start chasing
        }
        if (request.command == "end_episode") {
            //go to spawn location
        }
        if (request.command == "get_visibility"){
            response.command = "set_visibility";
            response.content << visibility_cone;
            send_data(response.to_json());
        }
        if (request.command == "set_game_state"){
            // new mouse or robot detection
            habitat_cv::Frame_detection state;
            request.content >> state;
        }
        // console
        if (request.command == "set_min_ghost_distance"){
            response.command = "result";
            if (set_ghost_min_distance(atoi(request.content.c_str()))){
                response.content = "success";
            } else {
                response.content = "failed";
            }
            send_data(response.to_json());
        }
        if (request.command == "set_world"){
            response.command = "result";
            if (set_world(request.content)) {
                response.content = "success";
            }else{
                response.content = "fail";
            }
            send_data(response.to_json());
        }

        if (request.command == "set_view_angle"){
            response.command = "result";
            if (set_view_angle(atof(request.content.c_str()) * M_PI * 2 / 360)) {
                response.content = "success";
            } else {
                response.content = "fail";
            }
            send_data(response.to_json());
        }

        if (request.command == "set_speed"){
            response.command = "result";
            if (set_speed(atof(request.content.c_str()))) {
                response.content = "success";
            } else {
                response.content = "fail";
            }
            send_data(response.to_json());
        }
    }

    void Robot_controller_service::on_disconnect()  {
        cout << "client disconnected "<< endl;
    }

    bool Robot_controller_service::set_world(const string &world_name) {
        try {
            auto new_data = new Data(world_name);
            auto old_data = data;
            data = new_data;
            free(old_data);
        } catch (...) {
            return false;
        }
        Occlusions occlusions;
        auto occluded_cells = data->cells.occluded_cells();
        for (const Cell &c:occluded_cells){
            occlusions.OcclusionIds.push_back(c.id);
        }
        return true;
    }

    bool Robot_controller_service::set_speed(double new_speed) {
        if (new_speed <= 0) return false;
        speed = new_speed;
        return true;
    }

    bool Robot_controller_service::set_view_angle(double new_view_angle) {
        if (view_angle <= 0) return false;
        view_angle = new_view_angle;
        return true;
    }

    bool Robot_controller_service::set_ghost_min_distance(unsigned int value) {
        min_ghost_distance = value;
        return update_spawn_locations() && set_spawn_cell();
    }

    int Robot_controller_service::port() {
        string port_str (std::getenv("CELLWORLD_ROBOT_CONTROLLER_PORT")?std::getenv("CELLWORLD_ROBOT_CONTROLLER_PORT"):"4000");
        return atoi(port_str.c_str());
    }

    bool Robot_controller_service::set_spawn_cell() {
        unsigned int spawn_cell_id;
        if (!spawn_locations.empty())
            spawn_cell_id = spawn_locations.random_cell().id;
        else
            spawn_cell_id = data->cells.free_cells().random_cell().id;
        return true;
    }

    bool Robot_controller_service::update_agent_state(const habitat_cv::Agent_info &info) {
        if (info.agent_name == "mouse") {
            prey.cell = data->map[info.coordinates];
            prey.location = info.location;
            prey.theta = info.theta;
            return true;
        } else if (info.agent_name == "robot") return false;

        predator.cell = data->map[info.coordinates];
        predator.location = info.location;
        predator.theta = info.theta;

        if (predator.cell && predator.cell){
            //plan
            if (control_robot(predator.location, predator.theta, prey.location)){
                // predator won
            }
        }
        return true;
    }

    bool Robot_controller_service::control_robot(Location location, double rotation, Location destination) {
        if (location.dist(destination) < 10) {
            robot.set_left(0);
            robot.set_right(0);
            robot.set_puf();
            robot.update();
            return true;
        }
        double destination_rotation = location.atan(destination);
        if (Visibility_cone::angle_difference(rotation, destination_rotation) < .1) {
            // facing in the right direction
            robot.set_left(80);
            robot.set_right(80);
            robot.update();
            return true;
        }
        if (Visibility_cone::direction(rotation, destination_rotation) == 0) {
            robot.set_left(-80);
            robot.set_right(80);
        } else {
            robot.set_left(80);
            robot.set_right(-80);
        }
        return true;
    }
}