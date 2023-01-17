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
        printf("rpy: %f %f %f\n", high_state_ros.imu.rpy[0], high_state_ros.imu.rpy[1], high_state_ros.imu.rpy[2]);

        motiontime += 2;

        high_cmd_ros.head[0] = 0xFE;
        high_cmd_ros.head[1] = 0xEF;
        high_cmd_ros.levelFlag = HIGHLEVEL;
        high_cmd_ros.mode = 0;
        high_cmd_ros.gaitType = 0;
        high_cmd_ros.speedLevel = 0;
        high_cmd_ros.footRaiseHeight = 0;
        high_cmd_ros.bodyHeight = 0;
        high_cmd_ros.euler[0] = 0;
        high_cmd_ros.euler[1] = 0;
        high_cmd_ros.euler[2] = 0;
        high_cmd_ros.velocity[0] = 0.0f;
        high_cmd_ros.velocity[1] = 0.0f;
        high_cmd_ros.yawSpeed = 0.0f;
        high_cmd_ros.reserve = 0;

        if (motiontime > 0 && motiontime < 2000)
        {
            high_cmd_ros.mode = 6;
        }
        else if(motiontime >= 2000 && motiontime < 3000)
        {
            high_cmd_ros.mode = 1;
        }
        else if(motiontime >= 3000 && motiontime < 4000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[0] = 0.3;
        }
        else if(motiontime >= 4000 && motiontime < 6000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[0] = -0.3;
        }
        else if(motiontime >= 6000 && motiontime < 8000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[1] = 0.3;
        }
        else if(motiontime >= 8000 && motiontime < 10000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[1] = -0.3;
        }
        else if(motiontime >= 10000 && motiontime < 12000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[2] = 0.3;
        }
        else if(motiontime >= 12000 && motiontime < 14000)
        {
            high_cmd_ros.mode = 1;
            high_cmd_ros.euler[2] = -0.3;
        }
        else if(motiontime >= 14000 && motiontime < 15000)
        {
            high_cmd_ros.mode = 1;
        }
        else if(motiontime >= 15000 && motiontime < 18000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.velocity[0] = 0.3;
            high_cmd_ros.yawSpeed = 0.2;
        }
        else if(motiontime >= 18000 && motiontime < 21000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.velocity[1] = -0.3;
            high_cmd_ros.yawSpeed = -0.2;
        }
        else if(motiontime >= 21000 && motiontime < 22000)
        {
            high_cmd_ros.mode = 1;
        }
        else if(motiontime >= 22000 && motiontime < 25000)
        {
            high_cmd_ros.mode = 2;
            high_cmd_ros.gaitType = 3;
        }
        else if(motiontime >= 25000 && motiontime < 26000)
        {
            high_cmd_ros.mode = 1;
        }
        else 
        {
            high_cmd_ros.mode = 0;
        }

        pub.publish(high_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}