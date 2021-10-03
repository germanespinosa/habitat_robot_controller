#pragma once
#include <cell_world.h>
#include <easy_tcp.h>
#include <fstream>
#include <cell_world_tools.h>

namespace cell_world::robot_controller {
    struct Agent_state{
        Cell cell = Cell::ghost_cell();
        Location location;
        double theta;
    };

    struct Robot_controller_service : easy_tcp::Service {
        struct Predator_instruction : json_cpp::Json_object {
            Json_object_members(
                    Add_member(destination);
                    Add_member(next_step);
                    Add_member(contact);
            );
            unsigned int destination = Cell::ghost_cell().id;
            unsigned int next_step = Cell::ghost_cell().id;
            bool contact;
        };

        void on_connect() override;
        void on_incoming_data(const std::string &plugin_data) override;
        void on_disconnect() override;

        static bool set_world(const std::string &);
        static bool set_speed(double);
        static bool set_view_angle(double);
        static bool update_spawn_locations();
        static bool set_ghost_min_distance(unsigned int);
        static bool set_spawn_cell();
        static bool update_agent_state(const cell_world::Agent_info  &);
        static bool control_robot(Location, double, Location);
        static int port();
    private:
        Cell_group visibility_cone;
    };
}