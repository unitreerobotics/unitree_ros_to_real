#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::LowState low_state_ros;

void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &state)
{
    static long count = 0;
    ROS_INFO("lowStateCallback %ld", count++);

    low_state_ros = *state;
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_postition_without_lcm");

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;
    ros::Rate loop_rate(500);

    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;

    unitree_legged_msgs::LowCmd low_cmd_ros;

    // bool initiated_flag = false; // initiate need time
    // int count = 0;

    ros::Publisher pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
    ros::Subscriber sub = nh.subscribe("low_state", 1, lowStateCallback);

    low_cmd_ros.levelFlag = LOWLEVEL;

    for (std::size_t i = 0; i < 20; ++i)
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

        printf("FR_joint_pos: %f %f %f\n", low_state_ros.motorState[FR_0].q, low_state_ros.motorState[FR_1].q, low_state_ros.motorState[FR_2].q);

        
        motiontime += 2;

        low_cmd_ros.motorCmd[FR_0].tau = -1.6f;
   
        if (motiontime >= 0)
        {
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
            if (motiontime >= 0 && motiontime < 10)
            {
                qInit[0] = low_state_ros.motorState[FR_0].q;
                qInit[1] = low_state_ros.motorState[FR_1].q;
                qInit[2] = low_state_ros.motorState[FR_2].q;
            }
            // second, move to the origin point of a sine movement with Kp Kd
            // if( motiontime >= 500 && motiontime < 1500){
            if (motiontime >= 10 && motiontime < 400)
            {
                rate_count++;
                double rate = rate_count / 200.0; // needs count to 200
            // Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0;
            // Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
                Kp[0] = 20.0;
                Kp[1] = 20.0;
                Kp[2] = 20.0;
                Kd[0] = 2.0;
                Kd[1] = 2.0;
                Kd[2] = 2.0;

                qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
            }
            double sin_joint1, sin_joint2;
        // last, do sine wave
            float freq_Hz = 1;
        // float freq_Hz = 5;
            float freq_rad = freq_Hz * 2 * M_PI;
            float t = dt * sin_count;
            if (motiontime >= 400)
            {
                sin_count++;
            // sin_joint1 = 0.6 * sin(3*M_PI*sin_count/1000.0);
            // sin_joint2 = -0.9 * sin(3*M_PI*sin_count/1000.0);
                sin_joint1 = 0.6 * sin(t * freq_rad);
                sin_joint2 = -0.9 * sin(t * freq_rad);
                qDes[0] = sin_mid_q[0];
                qDes[1] = sin_mid_q[1] + sin_joint1;
                qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];
            }

            low_cmd_ros.motorCmd[FR_0].q = qDes[0];
            low_cmd_ros.motorCmd[FR_0].dq = 0;
            low_cmd_ros.motorCmd[FR_0].Kp = Kp[0];
            low_cmd_ros.motorCmd[FR_0].Kd = Kd[0];
            low_cmd_ros.motorCmd[FR_0].tau = -1.6f;

            low_cmd_ros.motorCmd[FR_1].q = qDes[1];
            low_cmd_ros.motorCmd[FR_1].dq = 0;
            low_cmd_ros.motorCmd[FR_1].Kp = Kp[1];
            low_cmd_ros.motorCmd[FR_1].Kd = Kd[1];
            low_cmd_ros.motorCmd[FR_1].tau = 0.0f;

            low_cmd_ros.motorCmd[FR_2].q = qDes[2];
            low_cmd_ros.motorCmd[FR_2].dq = 0;
            low_cmd_ros.motorCmd[FR_2].Kp = Kp[2];
            low_cmd_ros.motorCmd[FR_2].Kd = Kd[2];
            low_cmd_ros.motorCmd[FR_2].tau = 0.0f;
        }
        

        pub.publish(low_cmd_ros);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
