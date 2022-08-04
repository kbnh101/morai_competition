#ifndef OBJECT_LIST_H
#define OBJECT_LIST_H
#include<iostream>
#include<ros/ros.h>

#include<morai_msgs/ObjectStatusList.h>

#include<geometry_msgs/Polygon.h>
#include<geometry_msgs/Point.h>

class object_list
{
public:
    object_list();
    morai_msgs::ObjectStatusList object;
    geometry_msgs::Polygon info;
    //Subscribe
    ros::Subscriber object_info;
    //Publisher
    ros::Publisher object_pub;

    void object_callback(const morai_msgs::ObjectStatusList &msg);

};

#endif // OBJECT_LIST_H
