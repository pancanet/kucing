#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
using namespace ros;


class kucing
{
public:
    kucing(NodeHandle& nodehandel);
    virtual ~kucing();

private:
    NodeHandle nodehandel;
    Publisher publisher;
    Publisher visual;
    Subscriber subcriber;
    void dataCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
    std::string topiq;
    int queque;
    float p_gain;
};