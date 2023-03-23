#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::HighState high_state_ros;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &state)
{
    static long count = 0;
    ROS_INFO("highStateCallback %ld", count++);
    high_state_ros = *state;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_walk_without_lcm");

    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;

    ros::Rate loop_rate(500);

    long motiontime = 0;

    unitree_legged_msgs::HighCmd high_cmd_ros;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    ros::Subscriber sub = nh.subscribe("high_state", 1000, highStateCallback);

    while (ros::ok())
    {
        std::cout << "motiontime:\t" << motiontime << " " << high_state_ros.imu.rpy[2] << "\n";

        motiontime += 2;

        high_cmd_ros.levelFlag = HIGHLEVEL;

        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.position[0] = 0.0f;
        high_cmd_ros.position[1] = 0.0f;
        high_cmd_ros.yawSpeed = 0.0f;

        high_cmd_ros.mode = 0;
        high_cmd_ros.rpy[0]  = 0;
        high_cmd_ros.rpy[1] = 0;
        high_cmd_ros.rpy[2] = 0;
        high_cmd_ros.gaitType = 0;
        high_cmd_ros.dBodyHeight = 0;
        high_cmd_ros.dFootRaiseHeight = 0;


        if (motiontime == 2)
        {
            std::cout<<"begin sending commands."<<std::endl;
        }
        if (motiontime>1000 && motiontime <2000)
        {
            high_cmd_ros.mode = 1;    // mode 1: force stand, control robot orientation and height
            high_cmd_ros.rpy[0] = 0.7;
        }
        if (motiontime>2000 && motiontime <3000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[0] = 0.;
        }
        if (motiontime>3000 && motiontime <4000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[1] = 0.4;
        }
        if (motiontime>4000 && motiontime <5000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[1] = 0.;
        }
        if (motiontime>5000 && motiontime <6000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[2] = 0.5;
        }
        if (motiontime>6000 && motiontime <7000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.rpy[2] = 0;
        }
        if (motiontime>8000 && motiontime <9000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.dBodyHeight = 0.05;
        }
        if (motiontime>9000 && motiontime <10000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.dBodyHeight = -0.15;
        }
        if (motiontime>10000 && motiontime <11000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.dBodyHeight = 0.;
        }
        if (motiontime>11000 && motiontime <13000)
        {
            high_cmd_ros.mode = 2;    // mode 2: following tareget velocity in body frame
            high_cmd_ros.gaitType = 0; // trot gait
            high_cmd_ros.velocity[0] = 0.2;
            high_cmd_ros.velocity[1] = 0;
            high_cmd_ros.yawSpeed = 0.3;
        }
        if (motiontime>13000 && motiontime <15000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gaitType = 0;
            high_cmd_ros.velocity[0] = -0.2;
            high_cmd_ros.velocity[1] = 0;
            high_cmd_ros.yawSpeed = -0.1;
            high_cmd_ros.dFootRaiseHeight = 0.1; // increase swing height
        }
        if (motiontime> 15000 && motiontime <17000)
        {
            high_cmd_ros.mode = 0; // normal trot of gaitType 0 will stop automatically
        }
        if (motiontime>17000 && motiontime <23000)
        {
            high_cmd_ros.mode = 3; // mode 3: walk to target position in ground frame
            high_cmd_ros.gaitType = 0;
            high_cmd_ros.speedLevel = 0; // adjust speedlevel
            high_cmd_ros.position[0] = 1;
            high_cmd_ros.position[1] = 0;
            high_cmd_ros.rpy[2] = 0;
            high_cmd_ros.dBodyHeight = -0.05; // lower body height
        }
        if (motiontime> 23000 && motiontime <25000)
        {
            high_cmd_ros.mode = 0; // normal trot of gaitType 0 will stop automatically
        }
        if (motiontime>25000 && motiontime <32000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gaitType = 2; // stairs walking gait
            high_cmd_ros.velocity[0] = 0.1; // avoid using higher forward speed than 0.37m/s for stair climbing gait
            high_cmd_ros.velocity[1] = 0.;
            high_cmd_ros.yawSpeed = 0;
        }
        if (motiontime>32000 && motiontime <40000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gaitType = 2;
            high_cmd_ros.velocity[0] = -0.1; // avoid using higher backward speed than -0.2m/s for stair climbing gait
            high_cmd_ros.velocity[1] = 0.;
            high_cmd_ros.yawSpeed = 0;
        }
        if (motiontime>40000 && motiontime <42000)
        {
            high_cmd_ros.mode = 1; // stairs walking gait will not automatically stop. change mode to force stand.
        }

        if (motiontime>42000 && motiontime <44000)
        {
            high_cmd_ros.mode = 5; // stand down
        }
        if (motiontime>44000 && motiontime <46000)
        {
            high_cmd_ros.mode = 6; // stand up
        }
        if (motiontime>46000 && motiontime <48000)
        {
            high_cmd_ros.mode = 5; // stand down
        }
        if (motiontime>48000 && motiontime <51000)
        {
            high_cmd_ros.mode = 7; // damping mode
        }
        if (motiontime>51000 && motiontime <53000)
        {
            high_cmd_ros.mode = 5; // stand down
        }
        if (motiontime>53000 && motiontime <55000)
        {
            high_cmd_ros.mode = 6; // stand up
        }
        if (motiontime>55000 && motiontime <57000)
        {
            high_cmd_ros.mode = 1; // force stand
        }
        if (motiontime>57000 && motiontime <65000)
        {
            high_cmd_ros.mode = 3; // mode 3: walk to target position in ground frame
            high_cmd_ros.gaitType = 0;
            high_cmd_ros.speedLevel = 0; // adjust speedlevel
            high_cmd_ros.position[0] = 0;
            high_cmd_ros.position[1] = 0;
            high_cmd_ros.rpy[2] = 0;
            high_cmd_ros.dBodyHeight = 0;
        }

        

        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}