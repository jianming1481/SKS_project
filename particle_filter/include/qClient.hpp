/*
 *   qClient.hpp
 *
 *   Date:2015
 *
 *   Author: Chien-Ming Lin
 *
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef Client_HPP_
#define Client_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include "std_msgs/Header.h"
#include "sensor_msgs/LaserScan.h"
#include <qt4/QtCore/QThread>
#include <string>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include <QFuture>
#include <qtconcurrentrun.h>

#define sensorlineNum 45

/*****************************************************************************
** Class
*****************************************************************************/

class Client : public QThread {

private:

    /****************************/
    /************Var*************/
    /****************************/
    double shift_x;
    double shift_y;
    double rotation;
    int *laser_dist;

    /****************************/
    /************ROS*************/
    /****************************/
    ros::NodeHandle *n;
    ros::Subscriber laser_sub;

public:
    Client(int argc, char** argv,const char* node_name);
    virtual ~Client() {}
    //void run();
    void ros_comms_init();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::Subscriber move_sub;

    int *return_laser_dist();
    geometry_msgs::Twist return_move();
};

#endif /* QClient_NODE_HPP_ */
