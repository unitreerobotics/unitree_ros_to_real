Packages Version: v3.2.1

# Introduction
This package can send control command to real robot from ROS. You can do low-level control(namely control all joints on robot) and high-level control(namely control the walking direction and speed of robot).

This version is suitable for unitree_legged_sdk v3.2 and v3.1, namely A1 and Aliengo robot. Currently, all the A1 and Aliengo use v3.2. If your Aliengo is really old, then it may use v3.1.
As for Go1, please use the v3.4 release version of this package and unitree_legged_sdk v3.4.

## Packages:

Basic message function: `unitree_legged_msgs`

The interface between ROS and real robot: `unitree_legged_real`

# Dependencies
* [unitree_legged_sdk](https://github.com/unitreerobotics): If your robot is suitable for `unitree_legged_sdk`, then you do not need `aliengo_sdk`.
* [aliengo_sdk](https://github.com/unitreerobotics): If your robot is suitable for `aliengo_sdk`, then you do not need `unitree_legged_sdk`.

# Configuration
Make sure the following exist in your `~/.bashrc` file or export them in terminal. `melodic`, `gazebo-8`, `~/catkin_ws`, `amd64` and the paths to `unitree_legged_sdk` should be replaced in your own case. 
If your use `unitree_legged_sdk`, then you need to set `UNITREE_SDK_VERSION=3_2` and the path `UNITREE_LEGGED_SDK_PATH`.
Otherwise, if you use `aliengo_sdk`, you need to set `UNITREE_SDK_VERSION=3_1` and the path `ALIENGO_SDK_PATH`.

```
source /opt/ros/melodic/setup.bash
source /usr/share/gazebo-8/setup.sh
source ~/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=~/catkin_ws:${ROS_PACKAGE_PATH}
export GAZEBO_PLUGIN_PATH=~/catkin_ws/devel/lib:${GAZEBO_PLUGIN_PATH}
export LD_LIBRARY_PATH=~/catkin_ws/devel/lib:${LD_LIBRARY_PATH}
# 3_1, 3_2
export UNITREE_SDK_VERSION=3_2
export UNITREE_LEGGED_SDK_PATH=~/unitree_legged_sdk
export ALIENGO_SDK_PATH=~/aliengo_sdk
# amd64, arm32, arm64
export UNITREE_PLATFORM="amd64"
```

# Build
You can use catkin_make to build ROS packages. First copy the package folder to `~/catkin_ws/src`, then:
```
cd ~/catkin_ws
catkin_make
```
Before compiling `unitree_legged_real`, please make sure that the `unitree_legged_msgs` has been compiled.

# Setup the net connection
First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, open the `ipconfig.sh` file under the folder `unitree_legged_real`, modify the port name to your own. And run the following commands:
```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```
If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.
In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name have to be changed to your own.

# Run the package
You can control your real robot(only A1 and Aliengo) from ROS by this package.

First you have to run the `real_launch` under root account:
```
sudo su
source /home/yourUserName/catkin_ws/devel/setup.bash
roslaunch unitree_legged_real real.launch rname:=a1 ctrl_level:=highlevel firmwork:=3_2
```
Please watchout that the `/home/yourUserName` means the home directory of yourself. These commands will launch a LCM server. The `rname` means robot name, which can be `a1` or `aliengo`(case does not matter), and the default value is `a1`. And the `ctrl_level` means the control level, which can be `lowlevel` or `highlevel`(case does not matter), and the default value is `highlevel`. Under the low level, you can control the joints directly. And under the high level, you can control the robot to move or change its pose. The `firmwork` means the firmwork version of the robot. The default value is `3_2` Now all the A1's firmwork version is `3_2`.

In order to send message to robot, you need to run the controller in another terminal(also under root account):
```
rosrun unitree_legged_real position_lcm
```
We offered some examples. When you run the low level controller, please make sure the robot is hanging up. The low level contains:
```
position_lcm
velocity_lcm
torque_lcm
```
The `velocity_lcm` and `torque_lcm` have to run under root account too. Please use the same method as runing `real_launch`.

And when you run the high level controller, please make sure the robot is standing on the ground. The high level only has `walk_lcm`.