/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include "unitree_legged_msgs/Cartesian.h"
#include "unitree_legged_msgs/HighCmd.h"
#include "unitree_legged_msgs/HighState.h"
#include "unitree_legged_msgs/IMU.h"
#include "unitree_legged_msgs/LED.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "ros/ros.h"


unitree_legged_msgs::Cartesian state2rosMsg(UNITREE_LEGGED_SDK::Cartesian &state)
{
    unitree_legged_msgs::Cartesian ros_msg;

    ros_msg.x = state.x;
    ros_msg.y = state.y;
    ros_msg.z = state.z;

    return ros_msg;
}

unitree_legged_msgs::IMU state2rosMsg(UNITREE_LEGGED_SDK::IMU &state)
{
    unitree_legged_msgs::IMU ros_msg;

    for (std::size_t i(0); i < 4; ++i)
        ros_msg.quaternion[i] = state.quaternion[i];
    
    for (std::size_t i(0); i < 3; ++i)
    {
        ros_msg.gyroscope[i] = state.gyroscope[i];
        ros_msg.accelerometer[i] = state.accelerometer[i];
        ros_msg.rpy[i] = state.rpy[i];
    }

    ros_msg.temperature = state.temperature;

    return ros_msg;
}

UNITREE_LEGGED_SDK::LED rosMsg2Cmd(const unitree_legged_msgs::LED &msg)
{
    UNITREE_LEGGED_SDK::LED cmd;
    cmd.r = msg.r;
    cmd.g = msg.g;
    cmd.b = msg.b;

    return cmd;
}

UNITREE_LEGGED_SDK::MotorCmd rosMsg2Cmd(const unitree_legged_msgs::MotorCmd &msg)
{
    UNITREE_LEGGED_SDK::MotorCmd cmd;

    cmd.mode = msg.mode;
    cmd.q = msg.q;
    cmd.dq = msg.dq;
    cmd.tau = msg.tau;
    cmd.Kp = msg.Kp;
    cmd.Kd = msg.Kd;

    for (std::size_t i(0); i < 3; ++i)
        cmd.reserve[i] = msg.reserve[i];
    

    return cmd;
}


unitree_legged_msgs::MotorState state2rosMsg(UNITREE_LEGGED_SDK::MotorState &state)
{
    unitree_legged_msgs::MotorState ros_msg;

    ros_msg.mode = state.mode;
    ros_msg.q = state.q;
    ros_msg.dq = state.dq;
    ros_msg.ddq = state.ddq;
    ros_msg.tauEst = state.tauEst;
    ros_msg.q_raw = state.q_raw;
    ros_msg.dq_raw = state.dq_raw;
    ros_msg.ddq_raw = state.ddq_raw;
    ros_msg.temperature = state.temperature;

    ros_msg.reserve[0] = state.reserve[0];
    ros_msg.reserve[1] = state.reserve[1];

    return ros_msg;
}

UNITREE_LEGGED_SDK::HighCmd rosMsg2Cmd(const unitree_legged_msgs::HighCmd &msg)
{
    UNITREE_LEGGED_SDK::HighCmd cmd;

    cmd.levelFlag = msg.levelFlag;
    cmd.commVersion = msg.commVersion;
    cmd.robotID = msg.robotID;
    cmd.SN = msg.SN;
    cmd.bandWidth = msg.bandWidth;
    cmd.mode = msg.mode;
    cmd.gaitType = msg.gaitType;
    cmd.speedLevel = msg.speedLevel;
    cmd.dFootRaiseHeight = msg.dFootRaiseHeight;
    cmd.dBodyHeight = msg.dBodyHeight;

    for (std::size_t i(0); i < 2; ++i)
    {
        cmd.position[i] = msg.position[i];
        cmd.velocity[i] = msg.velocity[i];
    }

    for (std::size_t i(0); i < 3; ++i)
        cmd.rpy[i] = msg.rpy[i];

    cmd.yawSpeed = msg.yawSpeed;
    
    for (std::size_t i(0); i < 4; ++i)
        cmd.led[i] = rosMsg2Cmd(msg.led[i]);
    

    for (std::size_t i(0); i < 40; ++i)
        cmd.wirelessRemote[i] = msg.wirelessRemote[i];
    
    cmd.reserve = msg.reserve;
    cmd.crc = msg.crc;

    return cmd;
}

unitree_legged_msgs::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state)
{
    unitree_legged_msgs::HighState ros_msg;

    ros_msg.levelFlag = state.levelFlag;
    ros_msg.commVersion = state.commVersion;
    ros_msg.robotID = state.robotID;
    ros_msg.SN = state.SN;
    ros_msg.bandWidth = state.bandWidth;
    ros_msg.mode = state.mode;
    ros_msg.imu = state2rosMsg(state.imu);

    for(std::size_t i(0); i < 3; ++i)
    {
        ros_msg.position[i] = state.position[i];
        ros_msg.velocity[i] = state.velocity[i];
    }

    ros_msg.yawSpeed = state.yawSpeed;

    for(std::size_t i(0); i < 4; ++i)
    {
        ros_msg.footPosition2Body[i] = state2rosMsg(state.footPosition2Body[i]);
        ros_msg.footSpeed2Body[i] = state2rosMsg(state.footSpeed2Body[i]);
        ros_msg.footForce[i] = state.footForce[i];
    }

    for (std::size_t i(0); i < 40; ++i)
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;
    

    return ros_msg;
}

unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_legged_msgs::LowState ros_msg;

    ros_msg.levelFlag = state.levelFlag;
    ros_msg.commVersion = state.commVersion;
    ros_msg.robotID = state.robotID;
    ros_msg.SN = state.SN;
    ros_msg.bandWidth = state.bandWidth;
    ros_msg.imu = state2rosMsg(state.imu);

    for (std::size_t i(0); i < 20; ++i)
        ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);

    for (std::size_t i(0); i < 4; ++i)
    {
        ros_msg.footForce[i] = state.footForce[i];
        ros_msg.footForceEst[i] = state.footForceEst[i];
    }
    
    ros_msg.tick = state.tick;

    for (std::size_t i(0); i < 40; ++i)
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];

    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;


    return ros_msg;
}


UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_legged_msgs::LowCmd &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    cmd.levelFlag = msg.levelFlag;
    cmd.commVersion = msg.commVersion;
    cmd.robotID = msg.robotID;
    cmd.SN = msg.SN;
    cmd.bandWidth = msg.bandWidth;

    for (std::size_t i(0); i < 20; ++i)
        cmd.motorCmd[i] = rosMsg2Cmd(msg.motorCmd[i]);
    

    for(std::size_t i(0); i < 4; ++i)
        cmd.led[i] = rosMsg2Cmd(msg.led[i]);

    for (std::size_t i(0); i < 40; ++i)
        cmd.wirelessRemote[i] = msg.wirelessRemote[i];

    cmd.reserve = msg.reserve;
    cmd.crc = msg.crc;

    return cmd;
}











#endif // _CONVERT_H_
