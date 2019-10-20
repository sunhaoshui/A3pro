# A3_pro

This is the inital unranged project code of the first "Zhi Sheng Kong Tian" UAV competition held by Air Force Engineering University

## Usage
Input the key of the osdk in the Inboard-SDK-ROS package.   
The main code is in Onboard-SDK-ROS/dji_sdk_demo/src/my_sdk.cpp

## Hardware
I give my hardware architecture as follows:


Item |Type|Note
:---:|:---:|:---:
Flight controller|DJI A3|
Remote control|DJI Datalink3|
Lidar|Hokuyo UST20LX|It is much more efficient to use lidar to search for  windows than camera
Camera|MVBlueFoxMLC200|
Camera|Intel Realsense D435i|This is can provide you with depth and imu information if you need|
Onboard computer|NUC8i7|
Onboard computer|Nvidia Jetson TX2|Can provide you with the GPU ability
Aircraft|DH410|A 410mm quadrotor and the mtors are from the TMOTOR|
Battery|ACE 4S 30C 5300mAh|

## Software
Cartographer: Use to locate the UAV in SE(2)  
VINS-Mono:Use to locate the UAV in SE(3)  
The version of the Onboard SDK used in this project is 3.6
