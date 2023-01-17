/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _CONVERT_H_
#define _CONVERT_H_

#include "unitree_legged_msgs/BmsCmd.h"
#include "unitree_legged_msgs/BmsState.h"
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

UNITREE_LEGGED_SDK::BmsCmd rosMsg2Cmd(const unitree_legged_msgs::BmsCmd &msg)
{
    UNITREE_LEGGED_SDK::BmsCmd cmd;

    cmd.off = msg.off;

    for (std::size_t i(0); i < 3; i++)
        cmd.reserve[i] = msg.reserve[i];
    

    return cmd;
}

unitree_legged_msgs::BmsState state2rosMsg(UNITREE_LEGGED_SDK::BmsState &state)
{
    unitree_legged_msgs::BmsState ros_msg;

    for (std::size_t i(0); i < 8; i++)
    {
        ros_msg.BQ_NTC[i] = state.BQ_NTC[i];
        ros_msg.MCU_NTC[i] = state.MCU_NTC[i];
    }

    for (std::size_t i(0); i < 30; i++)
        ros_msg.cell_vol[i] = state.cell_vol[i];
    

    ros_msg.version_h = state.version_h;
    ros_msg.version_l = state.version_l;
    ros_msg.bms_status = state.bms_status;
    ros_msg.SOC = state.SOC;
    ros_msg.current = state.current;
    ros_msg.cycle = state.cycle;

    return ros_msg;
}

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

    for (std::size_t i(0); i < 4; i++)
    {
        ros_msg.quaternion[i] = state.quaternion[i];
    }

    for (std::size_t i(0); i < 3; i++)
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

    for (std::size_t i(0); i < 3; i++)
    {
        cmd.reserve[i] = msg.reserve[i];
    }

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

    for (std::size_t i(0); i < 2; i++)
    {
        cmd.head[i] = msg.head[i];
        cmd.SN[i] = msg.SN[i];
        cmd.version[i] = msg.version[i];
        cmd.position[i] = msg.position[i];
        cmd.velocity[i] = msg.velocity[i];
        cmd.dComXy[i] = msg.dComXy[i];
        cmd.dstandFootXy[i] = msg.dstandFootXy[i];
    }

    for (std::size_t i(0); i < 3; i++)
        cmd.euler[i] = msg.euler[i];
    

    for (std::size_t i(0); i < 4; i++)
        cmd.led[i] = rosMsg2Cmd(msg.led[i]);
    

    for (std::size_t i(0); i < 40; i++)
        cmd.wirelessRemote[i] = msg.wirelessRemote[i];
    

    cmd.levelFlag = msg.levelFlag;
    cmd.frameReserve = msg.frameReserve;
    cmd.bandWidth = msg.bandWidth;
    cmd.mode = msg.mode;
    cmd.gaitType = msg.gaitType;
    cmd.speedLevel = msg.speedLevel;
    cmd.footRaiseHeight = msg.footRaiseHeight;
    cmd.bodyHeight = msg.bodyHeight;
    cmd.yawSpeed = msg.yawSpeed;
    cmd.reserve = msg.reserve;
    cmd.crc = msg.crc;

    cmd.bms = rosMsg2Cmd(msg.bms);

    return cmd;
}

unitree_legged_msgs::HighState state2rosMsg(UNITREE_LEGGED_SDK::HighState &state)
{
    unitree_legged_msgs::HighState ros_msg;

    for (std::size_t i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];
        ros_msg.SN[i] = state.SN[i];
        ros_msg.version[i] = state.version[i];
    }

    for (std::size_t i(0); i < 4; i++)
    {
        ros_msg.footForce[i] = state.footForce[i];
        ros_msg.footForceEst[i] = state.footForceEst[i];
        ros_msg.rangeObstacle[i] = state.rangeObstacle[i];
        ros_msg.footPosition2Body[i] = state2rosMsg(state.footPosition2Body[i]);
        ros_msg.footSpeed2Body[i] = state2rosMsg(state.footSpeed2Body[i]);
    }

    for (std::size_t i(0); i < 3; i++)
    {
        ros_msg.position[i] = state.position[i];
        ros_msg.velocity[i] = state.velocity[i];
    }

    for (std::size_t i(0); i < 40; i++)
    {
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    }

    for (std::size_t i(0); i < 20; i++)
    {
        ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    }

    ros_msg.imu = state2rosMsg(state.imu);

    ros_msg.bms = state2rosMsg(state.bms);

    ros_msg.levelFlag = state.levelFlag;
    ros_msg.frameReserve = state.frameReserve;
    ros_msg.bandWidth = state.bandWidth;
    ros_msg.mode = state.mode;
    ros_msg.progress = state.progress;
    ros_msg.gaitType = state.gaitType;
    ros_msg.footRaiseHeight = state.footRaiseHeight;
    ros_msg.bodyHeight = state.bodyHeight;
    ros_msg.yawSpeed = state.yawSpeed;
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;

    return ros_msg;
}

unitree_legged_msgs::LowState state2rosMsg(UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_legged_msgs::LowState ros_msg;

    for (std::size_t i(0); i < 2; i++)
    {
        ros_msg.head[i] = state.head[i];
        ros_msg.SN[i] = state.SN[i];
        ros_msg.version[i] = state.version[i];
    }

    for (std::size_t i(0); i < 4; i++)
    {
        ros_msg.footForce[i] = state.footForce[i];
        ros_msg.footForceEst[i] = state.footForceEst[i];
    }

    for (std::size_t i(0); i < 40; i++)
    {
        ros_msg.wirelessRemote[i] = state.wirelessRemote[i];
    }

    for (std::size_t i(0); i < 20; i++)
    {
        ros_msg.motorState[i] = state2rosMsg(state.motorState[i]);
    }

    ros_msg.imu = state2rosMsg(state.imu);

    ros_msg.bms = state2rosMsg(state.bms);

    ros_msg.tick = state.tick;
    ros_msg.reserve = state.reserve;
    ros_msg.crc = state.crc;

    return ros_msg;
}


UNITREE_LEGGED_SDK::LowCmd rosMsg2Cmd(const unitree_legged_msgs::LowCmd &msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;

    for (std::size_t i(0); i < 2; i++)
    {
        cmd.head[i] = msg.head[i];
        cmd.SN[i] = msg.SN[i];
        cmd.version[i] = msg.version[i];
    }

    for (std::size_t i(0); i < 40; i++)
    {
        cmd.wirelessRemote[i] = msg.wirelessRemote[i];
    }

    for (std::size_t i(0); i < 20; i++)
    {
        cmd.motorCmd[i] = rosMsg2Cmd(msg.motorCmd[i]);
    }

    cmd.bms = rosMsg2Cmd(msg.bms);

    cmd.levelFlag = msg.levelFlag;
    cmd.frameReserve = msg.frameReserve;
    cmd.bandWidth = msg.bandWidth;
    cmd.reserve = msg.reserve;
    cmd.crc = msg.crc;

    return cmd;
}











#endif // _CONVERT_H_
