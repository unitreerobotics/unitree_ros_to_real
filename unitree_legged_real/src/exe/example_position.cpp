#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_postition_without_lcm");

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;
    ros::Rate loop_rate(500);

    long motiontime = 0;
    int rate_count = 0;
    int sin_count = 0;
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};

    unitree_legged_msgs::LowCmd low_cmd_ros;

    bool initiated_flag = false; // initiate need time
    int count = 0;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);

    low_cmd_ros.head[0] = 0xFE;
    low_cmd_ros.head[1] = 0xEF;
    low_cmd_ros.levelFlag = LOWLEVEL;

    for (int i = 0; i < 12; i++)
    {
        low_cmd_ros.motorCmd[i].mode = 0x0A;  // motor switch to servo (PMSM) mode
        low_cmd_ros.motorCmd[i].q = PosStopF; // 禁止位置环
        low_cmd_ros.motorCmd[i].Kp = 0;
        low_cmd_ros.motorCmd[i].dq = VelStopF; // 禁止速度环
        low_cmd_ros.motorCmd[i].Kd = 0;
        low_cmd_ros.motorCmd[i].tau = 0;
    }

    while (ros::ok())
    {

        if (initiated_flag == true)
        {
            motiontime += 2;

            low_cmd_ros.motorCmd[FR_0].tau = -0.65f;
            low_cmd_ros.motorCmd[FL_0].tau = +0.65f;
            low_cmd_ros.motorCmd[RR_0].tau = -0.65f;
            low_cmd_ros.motorCmd[RL_0].tau = +0.65f;

            low_cmd_ros.motorCmd[FR_2].q = -M_PI / 2 + 0.5 * sin(2 * M_PI / 5.0 * motiontime * 1e-3);
            low_cmd_ros.motorCmd[FR_2].dq = 0.0;
            low_cmd_ros.motorCmd[FR_2].Kp = 5.0;
            low_cmd_ros.motorCmd[FR_2].Kd = 1.0;

            low_cmd_ros.motorCmd[FR_0].q = 0.0;
            low_cmd_ros.motorCmd[FR_0].dq = 0.0;
            low_cmd_ros.motorCmd[FR_0].Kp = 5.0;
            low_cmd_ros.motorCmd[FR_0].Kd = 1.0;

            low_cmd_ros.motorCmd[FR_1].q = 0.0;
            low_cmd_ros.motorCmd[FR_1].dq = 0.0;
            low_cmd_ros.motorCmd[FR_1].Kp = 5.0;
            low_cmd_ros.motorCmd[FR_1].Kd = 1.0;
        }

        count++;
        if (count > 10)
        {
            count = 10;
            initiated_flag = true;
        }

        pub.publish(low_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}