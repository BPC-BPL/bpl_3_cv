#include <memory>
#include "uav_control/drone_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    std::shared_ptr<DroneControl> drone_control_node = nullptr;
    std::string connection_url = "udp://:14541";

    try
    {
        drone_control_node = std::make_shared<DroneControl>("uav_control", connection_url, options);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "%s Exiting..\n", e.what());
        return 1;
    }

    exec.add_node(drone_control_node);

    exec.spin();
    rclcpp::shutdown();

    return 0;
}

