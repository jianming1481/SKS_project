#include "ros/ros.h"
#include "std_msgs/String.h"
#include "/home/iclab/catkin_ws/src/test_motor/include/test_motor/Motors.h"

#include <sstream>

using namespace Robot;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_motor");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);


  int device_id = 0;
  int device_id1 = 1;
  long d;
  Motors *m = new Motors();

  m->OpenDevice(device_id);
  m->SetEnable(device_id);
  m->ActivateProfileVelocityMode(device_id);


  m->OpenDevice(device_id1);
  m->SetEnable(device_id1);
  m->ActivateProfileVelocityMode(device_id1);

  m->SetVelocity(device_id,100);
  m->SetVelocity(device_id1,100);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;



//    ss << "hello world " << count;
//    msg.data = ss.str();



//    ROS_INFO("%s", msg.data.c_str());
//    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
//    ++count;
  }


  return 0;
}
