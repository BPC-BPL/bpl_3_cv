#include "bpl_waypoint_control/waypoint_control.hpp"

WaypointControl::WaypointControl(const std::string &node_name, const std::string &connection_url, const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    client_fly_to_point_ = this->create_client<bpl_interfaces::srv::Position>("fly_to_point");
    client_takeoff_ = this->create_client<std_srvs::srv::Trigger>("takeoff");
    client_land_ = this->create_client<std_srvs::srv::Trigger>("land");
    client_set_mode_ = this->create_client<std_srvs::srv::Trigger>("set_mode");

    // Define waypoints
    waypoints = {
        {-35.363161, 149.165137, 605.5586},
        {-35.363161, 149.165337, 605.5586},
        {-35.363361, 149.165337, 605.5586},
        {-35.363361, 149.165137, 605.5586},
        {-35.363161, 149.165137, 605.5586} 
    };
    RCLCPP_INFO(this->get_logger(), "Starting BPL Waypoint Control node...");
    callSetModeService();
    callTakeoffService();
    executeFlightPlan();
    callLandService();
}

void WaypointControl::executeFlightPlan()
{
    for (const auto &wp : waypoints)
    {
        if (!callFlyToPointService(wp.latitude_deg, wp.longitude_deg, wp.absolute_altitude_m))
        {
            RCLCPP_ERROR(this->get_logger(), "Flight aborted: Failed to reach waypoint.");
            return;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Flight plan complete.");
}

bool WaypointControl::callFlyToPointService(double latitude, double longitude, double altitude)
{
    auto request = std::make_shared<bpl_interfaces::srv::Position::Request>();
    request->latitude = latitude;
    request->longitude = longitude;
    request->altitude = altitude;

    RCLCPP_INFO(this->get_logger(), "Requesting flight to (%.6f, %.6f, %.2f)", latitude, longitude, altitude);

    auto result = client_fly_to_point_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint: %s", response->message.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service failed: %s", response->message.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call timeout or failure.");
    }
    return false;
}

bool WaypointControl::callTakeoffService()
{
    if (!client_takeoff_) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff service client is NULL!");
        return false;
    }

    if (!client_takeoff_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff service unavailable!");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_takeoff_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Takeoff successful.");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Takeoff failed: %s", response->message.c_str());
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Takeoff service timeout.");
    }
    return false;
}


bool WaypointControl::callLandService()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    RCLCPP_INFO(this->get_logger(), "Requesting landing...");
    auto result = client_land_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "Landing successful: %s", response->message.c_str());
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Landing failed: %s", response->message.c_str());
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Landing service call timeout.");
    }
    return false;
}

bool WaypointControl::callSetModeService()
{
    if (!client_set_mode_) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service client is NULL!");
        return false;
    }

    if (!client_set_mode_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Set mode service unavailable!");
        return false;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto result = client_set_mode_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Mode changed successfully.");
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Mode change failed: %s", response->message.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Mode change service did not respond in time.");
        return false;
    }
}



