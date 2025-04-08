#include "bpl_uav_control/drone_control.hpp"

DroneControl::DroneControl(const std::string &node_name, const std::string &connection_url, const rclcpp::NodeOptions &options)
    : Node(node_name, options)
{
    rclcpp::QoS qos(rclcpp::KeepLast(10)); // KeepLast(10) is often used for topics like battery status
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort); // Use BestEffort reliability

    _cb_group_subscribers = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    _cb_group_services = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = _cb_group_subscribers;
    
    _subBattery = this->create_subscription<sensor_msgs::msg::BatteryState>("/mavros/battery", qos, std::bind(&DroneControl::battery_callback, this, std::placeholders::_1));
    _subGps = this->create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global", qos, std::bind(&DroneControl::gps_callback, this, std::placeholders::_1));
    _subState = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", qos, std::bind(&DroneControl::state_callback, this, std::placeholders::_1));
    _subImu = this->create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data", qos, std::bind(&DroneControl::imu_callback, this, std::placeholders::_1));

    _pubSetpointGPS = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/mavros/setpoint_position/global", qos);

    _clientSetMode = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    _clientArming = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    _clientTakeoff = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    _clientLand = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");

    _srvTakeoff = this->create_service<std_srvs::srv::Trigger>(
        "takeoff", std::bind(&DroneControl::handle_takeoff, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, _cb_group_services);
    _srvLand = this->create_service<std_srvs::srv::Trigger>(
        "land", std::bind(&DroneControl::handle_land, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, _cb_group_services);
    _srvSetMode = this->create_service<std_srvs::srv::Trigger>(
        "set_mode", std::bind(&DroneControl::handle_set_mode, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, _cb_group_services);
    _srvFlyToPoint = this->create_service<bpl_interfaces::srv::Position>(
        "fly_to_point", std::bind(&DroneControl::handle_fly_to_point, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, _cb_group_services);
    
    _current_gps_position.latitude_deg = 0;
    _current_gps_position.longitude_deg = 0;
    _current_gps_position.absolute_altitude_m = 0;
}

void DroneControl::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg)    {
    /*RCLCPP_INFO(this->get_logger(), "Battery Voltage: %.2f V", msg->voltage);
    RCLCPP_INFO(this->get_logger(), "Battery Current: %.2f A", msg->current);
    RCLCPP_INFO(this->get_logger(), "Battery Percentage: %.2f%%", msg->percentage * 100.0);*/
}

void DroneControl::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    _current_gps_position.latitude_deg = msg->latitude;
    _current_gps_position.longitude_deg = msg->longitude;
    _current_gps_position.absolute_altitude_m = msg->altitude-20;
    //RCLCPP_INFO(this->get_logger(), "GPS Position: [Lat: %.6f, Lon: %.6f, Alt: %.2f]", _current_gps_position.latitude_deg = msg->latitude, _current_gps_position.longitude_deg, _current_gps_position.absolute_altitude_m);
}   

void DroneControl::state_callback(const mavros_msgs::msg::State::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "Flight Mode: %s", msg->mode.c_str());
}

void DroneControl::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "IMU Orientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
}

bool DroneControl::takeoff(double altitude)
{
    auto arm_request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_request->value = true;

    if (!_clientArming->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Arming service not available.");
        return false;  
    }

    auto arm_result = _clientArming->async_send_request(arm_request);
    arm_result.wait_for(std::chrono::seconds(5)); 

    if (arm_result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Drone armed successfully!");

        auto takeoff_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        takeoff_request->altitude = altitude;

        if (!_clientTakeoff->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Takeoff service not available.");
            return false;
        }

        auto takeoff_result = _clientTakeoff->async_send_request(takeoff_request);
        takeoff_result.wait_for(std::chrono::seconds(5)); 
        if (takeoff_result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Takeoff initiated successfully!");
            return true;  
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initiate takeoff!");
            return false; 
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone!");
        return false;  
    }
}



void DroneControl::land()
{
    auto land_request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    land_request->altitude = 0.0;  

    if (!_clientLand->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Land service not available.");
        return;
    }

    _clientLand->async_send_request(land_request, [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture result) {
        if (result.get()->success) {
            RCLCPP_INFO(this->get_logger(), "Landing initiated successfully!");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to initiate landing!");
        }
    });
}


void DroneControl::set_mode(const std::string &mode)
{
    auto mode_request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_request->custom_mode = mode;

    if (!_clientSetMode->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "SetMode service not available.");
        return;
    }

    _clientSetMode->async_send_request(mode_request, [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture result) {
        if (result.get()->mode_sent) {
            RCLCPP_INFO(this->get_logger(), "Mode set to '%s' successfully!", mode.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set mode to '%s'!", mode.c_str());
        }
    });
}


bool DroneControl::fly_to_point(double latitude, double longitude, double altitude)
{
    auto geo_pose = std::make_shared<geographic_msgs::msg::GeoPoseStamped>();
    geo_pose->pose.position.latitude = latitude;
    geo_pose->pose.position.longitude = longitude;
    geo_pose->pose.position.altitude = altitude;

    RCLCPP_INFO(this->get_logger(), "Flying to point (lat: %.6f, lon: %.6f, alt: %.2f)", latitude, longitude, altitude);

    _pubSetpointGPS->publish(*geo_pose);

    rclcpp::Rate rate(1); 
        while (true) {
            double distance = calculateDistance(latitude, longitude, altitude);
            RCLCPP_INFO(this->get_logger(), "Distance to target: %.2f meters", distance);
            if (distance <= 1.0) {
                return true;
            }
            rate.sleep();
        }
    return false;
}

void DroneControl::handle_takeoff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    double altitude = 10.0; 

    if (takeoff(altitude)) {
        rclcpp::Rate rate(1); 
        while (rclcpp::ok()) {
            double distance = 613-_current_gps_position.absolute_altitude_m-20;
            RCLCPP_INFO(this->get_logger(), "Distance to target: %.2f meters", distance);
            if (distance <= 0.1) {
                response->success = true;
                response->message = "Takeoff successful and target altitude of 10 meters reached.";
                return;
            }

            rate.sleep();
        }
        response->success = false;
        response->message = "Takeoff initiated, but target altitude not reached.";
    } else {
        response->success = false;
        response->message = "Failed to initiate takeoff.";
    }
}

void DroneControl::handle_land(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    land();
    response->success = true;
    response->message = "Landing initiated.";
}

void DroneControl::handle_set_mode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    set_mode("GUIDED");
    response->success = true;
    response->message = "Mode changed to GUIDED.";
}

void DroneControl::handle_fly_to_point(
    const std::shared_ptr<bpl_interfaces::srv::Position::Request> request,
    const std::shared_ptr<bpl_interfaces::srv::Position::Response> response)
{
    float latitude = request->latitude;
    float longitude = request->longitude;
    float altitude = request->altitude;
    if(fly_to_point(latitude, longitude, altitude)) {
        response->success = true;
        response->message = "Fly to point successful";
    }
    else    {
        response->success = false;
        response->message = "Fly to point not successful";
    }
    
}

double DroneControl::calculateDistance(float latitude, float longitude, float altitude) {
        const double earthRadiusKm = 6371.0;

        double lat1 = _current_gps_position.latitude_deg * M_PI / 180.0;
        double lon1 = _current_gps_position.longitude_deg * M_PI / 180.0;
        double lat2 = latitude * M_PI / 180.0;
        double lon2 = longitude * M_PI / 180.0;

        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        double a = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(lat1) * std::cos(lat2) * std::sin(dLon / 2) * std::sin(dLon / 2);
        double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

        double horizontal_distance = earthRadiusKm * c * 1000.0; 
        double vertical_distance = std::abs(_current_gps_position.absolute_altitude_m - altitude);

        double distance = std::sqrt(horizontal_distance * horizontal_distance + vertical_distance * vertical_distance);
        return distance;
}
