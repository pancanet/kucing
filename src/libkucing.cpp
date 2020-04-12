#include "kucing/libkucing.hpp"
#include <ros/ros.h>
using namespace ros;

kucing::kucing(NodeHandle &nodehandel) : nodehandel(nodehandel)
{
    if (!(nodehandel.getParam("/kucing/p_gain", p_gain)))
    {
        ROS_ERROR("P_GAIN ERROR");
        requestShutdown();
    }
    if (!(nodehandel.getParam("/kucing/topic_name", topiq)))
    {
        ROS_ERROR("TOPIC ERROR");
        requestShutdown();
    }
    if (!(nodehandel.getParam("/kucing/queue_size", queque)))
    {
        ROS_ERROR("queque ERROR");
        requestShutdown();
    }

    subcriber = nodehandel.subscribe(topiq, queque, &kucing::dataCallback, this);
    publisher = nodehandel.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    visual = nodehandel.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
    ROS_INFO("BERHASIL JALAN");
}
kucing::~kucing()
{
}

void kucing::dataCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    std::vector<float> laser_scan_range;
    laser_scan_range = msg->ranges;
    float angle_increment = msg->angle_increment;
    float angle_min = msg->angle_min;
    float radian_min;
    float x_min_pos, y_min_pos, z_min_pos;

    float min_distance = 10000;
    float min_index = 0;

    float zangle, gain;

    // Get minimum distance
    for (int i = 0; i < laser_scan_range.size(); i++)
    {
        if (min_distance > laser_scan_range[i])
        {
            min_distance = laser_scan_range[i];
            min_index = i;
        }
    }

    //ROS_INFO_STREAM("Minimum distance: " + std::to_string(min_distance));
    //start here
    radian_min = angle_min + angle_increment * min_index;
    x_min_pos = min_distance * cos(radian_min);
    y_min_pos = min_distance * sin(radian_min);
    z_min_pos = 1.0;

    //    ROS_INFO("x: %f, y: %f\n",x_min_pos, y_min_pos);

    if (min_distance > 1)
    {
        gain = kucing::p_gain * min_distance;
        zangle = kucing::p_gain * (0 - radian_min);
    }
    else
    {
        gain = 0;
        zangle = 0;
    }

    geometry_msgs::Twist velo_comd;
    velo_comd.linear.x = gain;
    velo_comd.angular.z = zangle;
    ROS_INFO("data: %f", gain);
    publisher.publish(velo_comd);

    // Publish visualization marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_laser";
    marker.header.stamp = Time();
    marker.ns = "base_laser";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x_min_pos;
    marker.pose.position.y = y_min_pos;
    marker.pose.position.z = z_min_pos;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    //Only if using a MESH_RESOURCE marker type:
    //    marker.mesh_resource = "package:///kucing/dae/base_link.dae";

    visual.publish(marker);
}