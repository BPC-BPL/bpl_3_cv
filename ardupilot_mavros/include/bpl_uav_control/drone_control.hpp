#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/battery_state.hpp"
#include "rclcpp/qos.hpp"
#include <std_srvs/srv/trigger.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include "bpl_interfaces/srv/position.hpp"
#include "bpl_uav_control/data_structures.hpp"

class DroneControl : public rclcpp::Node
{
public:
    DroneControl(const std::string &node_name, const std::string &connection_url, const rclcpp::NodeOptions &options);
    
private:
    rclcpp::CallbackGroup::SharedPtr _cb_group_subscribers;
    rclcpp::CallbackGroup::SharedPtr _cb_group_services;
    
    void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    bool takeoff(double altitude);
    void land();
    void set_mode(const std::string &mode);
    bool fly_to_point(double latitude, double longitude, double altitude);

    void handle_takeoff(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_land(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_set_mode(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void handle_fly_to_point(const std::shared_ptr<bpl_interfaces::srv::Position::Request> request, const std::shared_ptr<bpl_interfaces::srv::Position::Response> response);

    double calculateDistance(float latitude, float longitude, float altitude);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _subBattery;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _subGps;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr _subState;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _subImu;

    rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr _pubSetpointGPS;

    // Service clients
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _clientSetMode;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr _clientArming;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr _clientTakeoff;
    rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr _clientLand;

    // Service servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvTakeoff;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvLand;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvSetMode;
    rclcpp::Service<bpl_interfaces::srv::Position>::SharedPtr _srvFlyToPoint;

    Position _current_gps_position;
};


