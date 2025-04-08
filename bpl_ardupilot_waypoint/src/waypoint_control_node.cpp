#include <memory>
#include <string>
#include <iostream>
#include "bpl_waypoint_control/waypoint_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;

    rclcpp::NodeOptions options;
    std::string connection_url = "udp://:14540";

    std::shared_ptr<WaypointControl> waypoint_control_node = nullptr;

    try
    {
        waypoint_control_node = std::make_shared<WaypointControl>("waypoint_control", connection_url, options);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create WaypointControl node: %s", e.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    executor.add_node(waypoint_control_node);
    
    executor.spin();

    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
    return EXIT_SUCCESS;
}
