#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include <unitree_legged_msgs/MotorState.h>
#include <unitree_legged_msgs/BmsCmd.h>
#include <unitree_legged_msgs/BmsState.h>
#include <unitree_legged_msgs/IMU.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <ros/ros.h>

using namespace UNITREE_LEGGED_SDK;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{
    printf("yaw = %f\n", msg->imu.rpy[2]);
}

void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
    printf("FR_2_pos = %f\n", msg->motorState[FR_2].q);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_high_state_sub");

    ros::NodeHandle nh;

    unitree_legged_msgs::HighState high_state_ros;

    ros::Subscriber high_sub = nh.subscribe("high_state", 1, highStateCallback);
    ros::Subscriber low_sub = nh.subscribe("low_state", 1, lowStateCallback);

    ros::spin();

    return 0;
}