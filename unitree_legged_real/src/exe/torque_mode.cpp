/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

template<typename TLCM>
void* update_loop(void* param)
{
    TLCM *data = (TLCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

template<typename TCmd, typename TState, typename TLCM>
int mainHelper(int argc, char *argv[], TLCM &roslcm)
{
    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle n;
    ros::Rate loop_rate(500);

    long motiontime=0;
    float torque = 0;
    TCmd SendLowLCM = {0};
    TState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowROS;
    unitree_legged_msgs::LowState RecvLowROS;

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop<TLCM>, &roslcm);

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
    }

    while (ros::ok()){
        motiontime++;
        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);
        printf("FL_1 position: %f\n",  RecvLowROS.motorState[FL_1].q);

        // gravity compensation
        SendLowROS.motorCmd[FR_0].tau = -0.65f;
        SendLowROS.motorCmd[FL_0].tau = +0.65f;
        SendLowROS.motorCmd[RR_0].tau = -0.65f;
        SendLowROS.motorCmd[RL_0].tau = +0.65f;

        if( motiontime >= 500){
            torque = (0 - RecvLowROS.motorState[FL_1].q)*10.0f + (0 - RecvLowROS.motorState[FL_1].dq)*1.0f;
            if(torque > 5.0f) torque = 5.0f;
            if(torque < -5.0f) torque = -5.0f;

            SendLowROS.motorCmd[FL_1].q = PosStopF;
            SendLowROS.motorCmd[FL_1].dq = VelStopF;
            SendLowROS.motorCmd[FL_1].Kp = 0;
            SendLowROS.motorCmd[FL_1].Kd = 0;
            SendLowROS.motorCmd[FL_1].tau = torque;
        }
        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "torque_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);
    mainHelper<UNITREE_LEGGED_SDK::LowCmd, UNITREE_LEGGED_SDK::LowState, UNITREE_LEGGED_SDK::LCM>(argc, argv, roslcm);
}