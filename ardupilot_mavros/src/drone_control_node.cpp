#include <memory>
#include <string>
#include <iostream>
#include "bpl_uav_control/drone_control.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;

    // Node options and connection URL
    rclcpp::NodeOptions options;
    std::string connection_url = "udp://:14540";

    // Create the DroneControl node
    std::shared_ptr<DroneControl> drone_control_node = nullptr;

    try
    {
        drone_control_node = std::make_shared<DroneControl>("uav_control", connection_url, options);
    }
    catch (const std::exception &e)
    {
        // Log and exit on failure
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create DroneControl node: %s", e.what());
        rclcpp::shutdown();
        return EXIT_FAILURE;
    }

    // Add the node to the executor
    executor.add_node(drone_control_node);

    // Start spinning the executor
    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting MultiThreadedExecutor...");
    executor.spin();

    // Shutdown ROS 2
    rclcpp::shutdown();

    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
    return EXIT_SUCCESS;
}
