#include "ParticleFilter_node.hpp"

#define pNum 1
#define sensorlines 45
#define mapW 600
#define mapH 600


ParticleFilter pf(pNum,sensorlines,mapW,mapH);
paintImageClass image(pNum,sensorlines,mapW,mapH);

PFNode_Class::PFNode_Class(int argc, char **argv, const char *node_name):Client(argc,argv,node_name)
{
    this->pf_init();
}
void PFNode_Class::pf_init()
{
    this->ros_comms_init();
    pf.initParticle_Filter();
    pf.build_LikelihoodMap();
    image.readMap();
}

void PFNode_Class::pf_process()
{
    this->return_move();
}

void PFNode_Class::run()
{
    double Ismove;
    double Isrotate;
    geometry_msgs::Twist robot_shift;
    geometry_msgs::Twist robot_shift_tmp;

    init_robot(0) = 0;
    init_robot(1) = 300;
    init_robot(2) = 0;
    robot = init_robot;

    image.refresh_window(robot,pf.get_Particle(),pf.get_SensorWall(),pf.get_tpwall());
    while(ros::ok())
    {
        robot_shift = this->return_move();

        Ismove = sqrt((robot_shift_tmp.linear.x-robot_shift.linear.x)*(robot_shift_tmp.linear.x-robot_shift.linear.x)
                    + (robot_shift_tmp.linear.y-robot_shift.linear.y)*(robot_shift_tmp.linear.x-robot_shift.linear.y));
        Isrotate = fabs(robot_shift_tmp.angular.z-robot_shift.angular.z);

        if(Ismove>=5||Isrotate>0.087)
        {
            robot(0) = robot(0) + robot_shift.linear.x;
            robot(1) = robot(1) + robot_shift.linear.y;
            robot(2) = robot(2) + robot_shift.angular.z;

            pf.moveParticle(this->return_move());
            pf.Sim_SensorModel(robot);
            pf.rateGrade();
            image.refresh_window(robot,pf.get_Particle(),pf.get_SensorWall(),pf.get_tpwall());
        }
        robot_shift_tmp = robot_shift;
    }
}
