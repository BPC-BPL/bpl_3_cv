#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"

#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/battery_state.hpp"

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include "bpl_interfaces/srv/position.hpp"
#include "bpl_waypoint_control/data_structures.hpp"


class WaypointControl : public rclcpp::Node, public std::enable_shared_from_this<WaypointControl>
{
public:
    WaypointControl(const std::string &node_name, const std::string &connection_url, const rclcpp::NodeOptions &options);
    bool callFlyToPointService(double latitude, double longitude, double altitude);
    bool callTakeoffService();
    bool callLandService();
    bool callSetModeService();
    void executeFlightPlan();
    
private:  
    rclcpp::Client<bpl_interfaces::srv::Position>::SharedPtr client_fly_to_point_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_takeoff_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_land_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_set_mode_;
    Position _current_gps_position;
    std::vector<Position> waypoints;

    
};


