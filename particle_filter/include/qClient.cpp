/*
 *   qClient.cpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <sstream>
#include "../include/qClient.hpp"
#include <std_msgs/String.h>



/*****************************************************************************
** ROS
*****************************************************************************/

Client::Client(int argc, char** argv,const char* node_name)
{
    std::cout << "Initializing Node...\n";
    ros::init(argc,argv,node_name);
    ROS_INFO("Connected to roscore");
}
void Client::ros_comms_init() {
    n = new ros::NodeHandle();
    laser_sub = n->subscribe("/scan",1,&Client::laserCallback,this);
    move_sub = n->subscribe("/sks_speed_topic",1,&Client::moveCallback,this);
    //ros::spin();
}

/*void Client::run() {
    ros::Rate loop_rate(1000);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}*/
void Client::moveCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
        shift_x = msg->linear.x;
        shift_y = msg->linear.y;
        rotation = msg->angular.z;
        //ROS_INFO("go");
}
void Client::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i=0;i<180;i++)
    {
        ROS_INFO("%d:[%0.2f]",i,msg->ranges[(i)*4]*100);
    }
//    ros::Rate loop_rate(1000);
//    ros::spinOnce();
//    loop_rate.sleep();
}

/*****************************************************************************
** Return Value
*****************************************************************************/
geometry_msgs::Twist Client::return_move()
{
    geometry_msgs::Twist tmp;
    tmp.linear.x = shift_x;
    tmp.linear.y = -shift_y;
    tmp.angular.z = rotation;
    //ROS_INFO("x:%f\ty:%f",shift_x,shift_y);
    return tmp;
}
