#ifndef MY_SDK_H
#define MY_SDK_H
// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>


#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include "pid.h"

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void height_above_takeoff_callback(const std_msgs::Float32::ConstPtr& msg);

void guidance_position_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void guidance_ultrasonic_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

void cartographer2D_callback(const tf2_msgs::TFMessage::ConstPtr& msg);

void ust20lx_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool A3monitoredTakeoff();

bool A3TakeoffWithoutGPS();

bool M100monitoredTakeoff();

bool set_local_position();

bool set_control(float x, float y, float z, float yaw, uint8_t flag);

float get_time_interval(ros::Time begin_time);

geometry_msgs::Point rotation_coordinate_inverse(geometry_msgs::Quaternion q, geometry_msgs::Point Body);

geometry_msgs::Point rotation_coordinate(geometry_msgs::Quaternion q, geometry_msgs::Point ENU);

void PIDX_velocity_control(PID PIDinput,float pos_value);

void PIDVx_Degree_control(PID PIDinputP, float pos_value);

void PIDY_velocity_control(PID PIDinput, float pos_value);

void PIDZ_velocity_control(PID PIDinput, float pos_value);

void PIDX_velocity_control_guidance(PID PIDinput, float pos_value);

void PIDY_velocity_control_guidance(PID PIDinput,float pos_value, float error_input, ros::Time time_now);

void PIDZ_velocity_control_guidance(PID PIDinput, float pos_value);

void PIDX_velocity_control_cartographer2D(PID PIDinput, float pos_value);

void PIDY_velocity_control_cartographer2D(PID PIDinput, float pos_value);

void PIDXY_velocity_control_cartographer2D(PID PIDinput_x, PID PIDinput_y, float pos_value_x, float pos_value_y);

void PIDXY_Traj_velocity_control(PID PIDinput, PID PIDinputYaw, float pos_value_x, float pos_value_y);

void PIDXYYaw_velocity_control(PID PIDinput, PID PIDinputYaw, float pos_value_x, float pos_value_y, float yaw_value);

void PIDYaw_Velocity_control(PID PIDinput,float yaw_value);

void PIDXY_Traj_degree_control(PID PIDinput, PID PIDinputYaw, float pos_value_x, float pos_value_y);

void PIDZ_thrust_control(PID PidinputP, PID PidinputV, float pos_value_z);

void test_thrust_balance(float start, float end, float dt);

bool move_to_saferegine(PID PID_input);

void move_to_door(PID PID_input);

void enter_puzzle(PID PID_input);

#endif // MY_SDK_H
