#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>

Custom custom;

ros::Subscriber sub_high;
ros::Subscriber sub_low;

ros::Publisher pub_high;
ros::Publisher pub_low;

long high_count = 0;
long low_count = 0;

void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    printf("highCmdCallback is running !\t%ld\n", ::high_count);

    custom.cmdAssignment(msg);

    unitree_legged_msgs::HighState high_state_ros;

    custom.stateAssignment(high_state_ros);

    pub_high.publish(high_state_ros);

    printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
}

void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{

    printf("lowCmdCallback is running !\t%ld\n", low_count);

    custom.cmdAssignment(msg);

    unitree_legged_msgs::LowState low_state_ros;

    custom.stateAssignment(low_state_ros);

    pub_low.publish(low_state_ros);

    printf("lowCmdCallback ending!\t%ld\n\n", ::low_count++);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_udp");

    ros::NodeHandle nh;

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        sub_low = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

        LoopFunc loop_udpSend("low_udp_send", 0.002, 3, boost::bind(&Custom::lowUdpSend, &custom));
        LoopFunc loop_udpRecv("low_udp_recv", 0.002, 3, boost::bind(&Custom::lowUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        ros::spin();

        // printf("low level runing!\n");
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        sub_high = nh.subscribe("high_cmd", 1, highCmdCallback);
        pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

        LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
        LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));

        loop_udpSend.start();
        loop_udpRecv.start();

        ros::spin();

        // printf("high level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0;
}