#pragma once

#include <iostream>
#include "geometry_msgs/msg/point.hpp"


struct Position
{
    double latitude_deg;
    double longitude_deg;
    float absolute_altitude_m;
    float relative_altitude_m;
};

struct EulerAngle
{
    float roll;
    float pitch;
    float yaw;
};

struct Battery
{
    float voltage;
    float current;
    float remaining_percent;
};

struct GpsInfo
{
    int num_of_sattelites;
    int fix_type;
};

struct RcStatus
{
    bool was_available_once;
    bool is_available;
    float signal_strenght_percent;
};

struct PositionXYZ
{
    float x;
    float y;
    float z;

    PositionXYZ& operator=(const geometry_msgs::msg::Point& point)
    {
        x = point.x;
        y = point.y;
        z = point.z;
        return *this;
    }
};