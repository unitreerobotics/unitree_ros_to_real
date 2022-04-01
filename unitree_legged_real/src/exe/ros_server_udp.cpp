/************************************************************************
Copyright (c) 2020, Kei Okada. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ros/ros.h>
#include <pthread.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <geometry_msgs/Twist.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;
// High command Lcm Server
class Ros_Server_High
{
public:
  Ros_Server_High(uint16_t local_port, const char* target_ip, uint16_t target_port): udp(local_port, target_ip, target_port, sizeof(HighCmd), sizeof(HighState)){
    pub = n.advertise<unitree_legged_msgs::HighState>("high_state", 1);
    sub = n.subscribe<unitree_legged_msgs::HighCmd>("high_cmd", 1, [&](const unitree_legged_msgs::HighCmdConstPtr& msg){SendHighROS=*msg;});
    sub_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &Ros_Server_High::CmdVelCB, this);

    udp.InitCmdData(cmd);
  }
  void CmdVelCB(const geometry_msgs::TwistConstPtr& msg){
    SendHighROS.mode = 2;  // 2. target velocity walking (controlled by velocity + yawSpeed)
    SendHighROS.velocity[0] = msg->linear.x;
    SendHighROS.velocity[1] = msg->linear.y;
    SendHighROS.yawSpeed = msg->angular.z;
  }
  void UDPRecv(){
    udp.Recv();
  }
  void UDPSend(){
    udp.Send();
  }
  void RobotControl();

  UDP udp;
  HighCmd cmd = {0};
  HighState state = {0};

  ros::NodeHandle n;
  ros::Publisher pub;
  ros::Subscriber sub, sub_vel;
  unitree_legged_msgs::HighCmd SendHighROS;
  unitree_legged_msgs::HighState RecvHighROS;
};


void Ros_Server_High::RobotControl()
{
    udp.GetRecv(state);
    RecvHighROS = ToRos(state);
    pub.publish(RecvHighROS);
    ROS_INFO_THROTTLE(1, "** STATE **");
    ROS_INFO_THROTTLE(1, "head                %8d %8d", state.head[0], state.head[0]);
    ROS_INFO_THROTTLE(1, "levelFlag           %8d", state.levelFlag);
    ROS_INFO_THROTTLE(1, "SN                  %8d %8d", state.SN[0], state.SN[1]);
    ROS_INFO_THROTTLE(1, "version             %8d %8d", state.version[0], state.version[0]);
    ROS_INFO_THROTTLE(1, "bandWidth           %8d", state.bandWidth);
    ROS_INFO_THROTTLE(1, "imu.temperature     %d", state.imu.temperature);
    ROS_INFO_THROTTLE(1, "imu.quaternion      %f %f %f %f", state.imu.quaternion[0], state.imu.quaternion[1], state.imu.quaternion[2], state.imu.quaternion[3]);
    ROS_INFO_THROTTLE(1, "imu.gyroscope       %f %f %f", state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
    ROS_INFO_THROTTLE(1, "imu.accelerometer   %f %f %f", state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
    ROS_INFO_THROTTLE(1, "bms                 %d", state.bms.bms_status);
    ROS_INFO_THROTTLE(1, "footForce           %d %d %d %d", state.footForce[0], state.footForce[1], state.footForce[2], state.footForce[3]);
    ROS_INFO_THROTTLE(1, "footForceEst        %d %d %d %d", state.footForceEst[0], state.footForceEst[1], state.footForceEst[2], state.footForceEst[3]);
    ROS_INFO_THROTTLE(1, "mode                %8d", state.mode);
    ROS_INFO_THROTTLE(1, "progress            %f", state.progress);
    ROS_INFO_THROTTLE(1, "gaitType            %d", state.gaitType);
    ROS_INFO_THROTTLE(1, "footRaiseHeight     %f", state.footRaiseHeight);
    ROS_INFO_THROTTLE(1, "position            %f %f", state.position[0], state.position[1]);
    ROS_INFO_THROTTLE(1, "bodyHeight          %f", state.bodyHeight);
    ROS_INFO_THROTTLE(1, "velocity            %f %f", state.velocity[0], state.velocity[1]);
    ROS_INFO_THROTTLE(1, "yawSpeed            %f", state.yawSpeed);
    ROS_INFO_THROTTLE(1, "rangeObstacle       %f %f %f %f", state.rangeObstacle[0], state.rangeObstacle[1], state.rangeObstacle[2], state.rangeObstacle[3]);
    ROS_INFO_THROTTLE(1, "footPosition2Body   (%f %f %f) (%f %f %f) (%f %f %f) (%f %f %f)", state.footPosition2Body[0].x, state.footPosition2Body[0].y, state.footPosition2Body[0].z, state.footPosition2Body[1].x, state.footPosition2Body[1].y, state.footPosition2Body[1].z, state.footPosition2Body[2].x, state.footPosition2Body[2].y, state.footPosition2Body[2].z, state.footPosition2Body[3].x, state.footPosition2Body[3].y, state.footPosition2Body[3].z);
    ROS_INFO_THROTTLE(1, "footSpeed2Body      (%f %f %f) (%f %f %f) (%f %f %f) (%f %f %f)", state.footSpeed2Body[0].x, state.footSpeed2Body[0].y, state.footSpeed2Body[0].z, state.footSpeed2Body[1].x, state.footSpeed2Body[1].y, state.footSpeed2Body[1].z, state.footSpeed2Body[2].x, state.footSpeed2Body[2].y, state.footSpeed2Body[2].z, state.footSpeed2Body[3].x, state.footSpeed2Body[3].y, state.footSpeed2Body[3].z);
    //ROS_INFO_THROTTLE(1, "wirelessRemote     ");for(int i = 0; i < 40; i++) ROS_INFO_THROTTLE(1, " %d", state.wirelessRemote[i]); ROS_INFO_THROTTLE(1, "");
    ROS_INFO_THROTTLE(1, "reserve             %d", state.reserve);
    ROS_INFO_THROTTLE(1, "crc                 %d", state.crc);


    // need to keep original cmd.head to move robots
    std::array<uint8_t, 2> head = cmd.head;
    cmd = ToLcm(SendHighROS, cmd);
    cmd.head = head;

    //
    ROS_INFO_THROTTLE(1, "** COMMAND **");
    ROS_INFO_THROTTLE(1, "head                %8d %8d", cmd.head[0], cmd.head[1]);
    ROS_INFO_THROTTLE(1, "levelFlag           %8d", cmd.levelFlag);
    ROS_INFO_THROTTLE(1, "frameReserve        %8d", cmd.frameReserve);
    ROS_INFO_THROTTLE(1, "SN                  %8d %8d", cmd.SN[0], cmd.SN[1]);
    ROS_INFO_THROTTLE(1, "version             %8d %8d", cmd.version[0], cmd.version[1]);
    ROS_INFO_THROTTLE(1, "bandWidth           %8d", cmd.bandWidth);
    ROS_INFO_THROTTLE(1, "mode                %8d", cmd.mode);
    ROS_INFO_THROTTLE(1, "gaitType            %d", cmd.gaitType);
    ROS_INFO_THROTTLE(1, "speedLevel          %d", cmd.speedLevel);
    ROS_INFO_THROTTLE(1, "footRaiseHeight     %f", cmd.footRaiseHeight);
    ROS_INFO_THROTTLE(1, "bodyHeight          %f", cmd.bodyHeight);
    ROS_INFO_THROTTLE(1, "position            %f %f", cmd.postion[0], cmd.postion[1]);
    ROS_INFO_THROTTLE(1, "euler               %f %f %f", cmd.euler[0], cmd.euler[1], cmd.euler[2]);
    ROS_INFO_THROTTLE(1, "velocity            %f %f", cmd.velocity[0], cmd.velocity[1]);
    ROS_INFO_THROTTLE(1, "yawSpeed            %f", cmd.yawSpeed);
    ROS_INFO_THROTTLE(1, "bms                 %d", cmd.bms.off);
    ROS_INFO_THROTTLE(1, "led                 (%d %d %d)", cmd.led[0].r,  cmd.led[0].g,  cmd.led[0].b);
    ROS_INFO_THROTTLE(1, "                    (%d %d %d)", cmd.led[1].r,  cmd.led[1].g,  cmd.led[1].b);
    ROS_INFO_THROTTLE(1, "                    (%d %d %d)", cmd.led[2].r,  cmd.led[2].g,  cmd.led[2].b);
    ROS_INFO_THROTTLE(1, "                    (%d %d %d)", cmd.led[3].r,  cmd.led[3].g,  cmd.led[3].b);
    //ROS_INFO_THROTTLE(1, "wirelessRemote     ");for(int i = 0; i < 40; i++) ROS_INFO_THROTTLE(1, " %d", cmd.wirelessRemote[i]); ROS_INFO_THROTTLE(1, "");
    ROS_INFO_THROTTLE(1, "reserve             %d", cmd.reserve);
    ROS_INFO_THROTTLE(1, "crc                 %d", cmd.crc);

    //
    udp.SetSend(cmd);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "ros_server");

    int local_port = 8090;
    std::string target_ip = std::string("192.168.123.161");
    int target_port = 8082;
    ros::param::get("~local_port", local_port);
    ros::param::get("~target_ip", target_ip);
    ros::param::get("~target_port", target_port);

    Ros_Server_High server(local_port, target_ip.c_str(), target_port);
    LoopFunc loop_control("control_loop", 0.002, boost::bind(&Ros_Server_High::RobotControl, &server));
    LoopFunc loop_udpSend("UDP_Send", 0.002, 3, boost::bind(&Ros_Server_High::UDPSend, &server));
    LoopFunc loop_udpRecv("UDP_Recv", 0.002, 3, boost::bind(&Ros_Server_High::UDPRecv, &server));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();
}
