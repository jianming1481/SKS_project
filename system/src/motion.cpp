/*****************************************
//                               y
//       \       /               ^
//        \     /                |
//         \   /                 |
//           |             ------------> x
//           |                   |
//           |                   |
//
/****************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "/home/iclab/catkin_ws/src/system/include/Motors.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

#define WHEEL_RADIUS 0.05125
#define ROBOT_RADIUS 0.17
#define PULSE_PER_TURN 4096.0
#define GEAR_RATIO 2.85

#define debug 0

#define ANGLE_1 M_PI/6
#define ANGLE_2 5*(M_PI/6)
#define ANGLE_3 3*(M_PI/2)
#define mDistConst ((2*WHEEL_RADIUS*M_PI)/(PULSE_PER_TURN*GEAR_RATIO))

using namespace Robot;

long MotorPlus[3];
long PastMotorPlus[3];

double w1,w2,w3;
double pulse_diff;
double mWheelDist[3];

const double mAngle1Sin(sin(M_PI/6));
const double mAngle2Sin(sin(5*M_PI/6));
const double mAngle3Sin(sin(3*M_PI/2));

const double mAngle1Cos(cos(M_PI/6));
const double mAngle2Cos(cos(5*M_PI/6));
const double mAngle3Cos(cos(3*M_PI/2));

geometry_msgs::Twist movement;
int motor_ID0,motor_ID1,motor_ID2;


/*==============================================================================*/
//Initialize
/*==============================================================================*/
void Initialize()
{
    w1=0;
    w2=0;
    w3=0;
    motor_ID0 = 0;
    motor_ID1 = 1;
    motor_ID2 = 2;
    PastMotorPlus[0] = 0;
    PastMotorPlus[1] = 0;
    PastMotorPlus[2] = 0;

}
/*==============================================================================*/
//Get motor distance
/*==============================================================================*/
void GetWheelDist(short motor_ID)
{
    if(PastMotorPlus[motor_ID] == 0)
        PastMotorPlus[motor_ID] = MotorPlus[motor_ID];
    pulse_diff = (MotorPlus[motor_ID] - PastMotorPlus[motor_ID]);
    PastMotorPlus[motor_ID] = MotorPlus[motor_ID];
    mWheelDist[motor_ID] = pulse_diff*mDistConst;
    if(debug)
    {
        std::cout << "pulse_diff[" << motor_ID << "]:" << pulse_diff << std::endl;
    }
}

/*==============================================================================*/
//Forward Kinematics
/*==============================================================================*/
void Kinematics()
{
    //calculate wheeled distance
    GetWheelDist(motor_ID0);
    GetWheelDist(motor_ID1);
    GetWheelDist(motor_ID2);

    movement.linear.x += 0.5774*mWheelDist[0] + (-0.5774)*mWheelDist[1] + 0.0*mWheelDist[2];
    movement.linear.y += 0.3333*mWheelDist[0] + 0.3333*mWheelDist[1] + (-0.6667)*mWheelDist[2];
    movement.angular.z += 1.9608*mWheelDist[0] + 1.9608*mWheelDist[1] + 1.9608*mWheelDist[2];
}

/*==============================================================================*/
//Topic call back
/*==============================================================================*/
void motionCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    w1= mAngle1Cos*msg->linear.x + mAngle1Sin*msg->linear.y + ROBOT_RADIUS*msg->angular.z;
    w2= mAngle2Cos*msg->linear.x + mAngle2Sin*msg->linear.y + ROBOT_RADIUS*msg->angular.z;
    w3= mAngle3Cos*msg->linear.x + mAngle3Sin*msg->linear.y + ROBOT_RADIUS*msg->angular.z;

    if(debug)
    {
        std::cout << "v_x:" << msg->linear.x << std::endl;
        std::cout << "v_y:" << msg->linear.y << std::endl;
        std::cout << "rot:" << msg->angular.z << std::endl;
        std::cout << "w1:" << w1 << std::endl;
        std::cout << "w2:" << w2 << std::endl;
        std::cout << "w3:" << w3 << std::endl;
    }

    int times = 5;
    w1 = w1*times;
    w2 = w2*times;
    w3 = w3*times;
}
/*==============================================================================*/
//Main
/*==============================================================================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion");

  ros::NodeHandle n;
  ros::Subscriber motion_sub = n.subscribe("/sks_speed_topic",1,motionCallback);
  Motors *m = new Motors();
  Initialize();

  //Open motor_0
  m->OpenDevice(motor_ID0);
  m->SetEnable(motor_ID0);
  m->ActivateProfileVelocityMode(motor_ID0);

  //Open motor_1
  m->OpenDevice(motor_ID1);
  m->SetEnable(motor_ID1);
  m->ActivateProfileVelocityMode(motor_ID1);

  //Open motor_2
  m->OpenDevice(motor_ID2);
  m->SetEnable(motor_ID2);
  m->ActivateProfileVelocityMode(motor_ID2);

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
      //set motor velocity
      m->SetVelocity(motor_ID0,w1);
      m->SetVelocity(motor_ID1,w2);
      m->SetVelocity(motor_ID2,w3);

      //get plus and store in MotorPlus
      m->GetPositionIs(motor_ID0, &MotorPlus[motor_ID0]);
      m->GetPositionIs(motor_ID1, &MotorPlus[motor_ID1]);
      m->GetPositionIs(motor_ID2, &MotorPlus[motor_ID2]);

      std::cout << "x:" << movement.linear.x << std::endl;
      std::cout << "y:" << movement.linear.y << std::endl;
      std::cout << "yaw:" << movement.angular.z*180/M_PI << std::endl;

      Kinematics();

      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}

