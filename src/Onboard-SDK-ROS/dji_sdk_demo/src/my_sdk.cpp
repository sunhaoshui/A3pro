/** @file my_sdk.cpp
 *  @version 3.6
 *  @date Nov, 2018
 *  @author Sun Haoshui
 *
 *  @brief
 *  program of the A3 control for Zhishengkongtian
 *
 *  @copyright 2018 MEICATA. All rights reserved.
 *
 */

#include "my_sdk.h"
#include "dji_sdk/dji_sdk.h"
#include "param.h"
#include "math.h"



float cruse_height = 2.0;


const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

uint8_t velocity_control_flag = (DJISDK::VERTICAL_VELOCITY   |
            DJISDK::HORIZONTAL_VELOCITY |
            DJISDK::YAW_RATE            |
            DJISDK::HORIZONTAL_BODY     |
            DJISDK::STABLE_ENABLE);

uint8_t degree_control_flag = (DJISDK::VERTICAL_VELOCITY   |
            DJISDK::HORIZONTAL_ANGLE    |
            DJISDK::YAW_RATE            |
            DJISDK::HORIZONTAL_BODY     |
            DJISDK::STABLE_ENABLE);

uint8_t thrust_control_flag = (DJISDK::VERTICAL_THRUST   |
            DJISDK::HORIZONTAL_ANGLE    |
            DJISDK::YAW_RATE            |
            DJISDK::HORIZONTAL_BODY     |
            DJISDK::STABLE_ENABLE);

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

uint8_t flight_status = 255;
uint8_t display_mode  = 255;

ros::Publisher ctrlVelYawPub;

geometry_msgs::Quaternion current_atti;
geometry_msgs::Quaternion inital_atti;

sensor_msgs::NavSatFix current_gps;
geometry_msgs::Point current_local_pos;

geometry_msgs::Vector3Stamped current_vel;

geometry_msgs::Point cartographer2D_pos;
geometry_msgs::Quaternion cartographer2D_q;

geometry_msgs::Vector3Stamped guidance_pos;
sensor_msgs::LaserScan guidance_ultrosonic;

sensor_msgs::LaserScan ust20lx;

float yaw2, yaw1 = 0;

float height; //the height of the plane in m

float guidance_height = 0.0;//the height of the plane in m taken by guidance
float guidance_height_past = 0.0;

const float error_thousold = 0.05;
const float error_thousold_yaw = 0.1;

float quad_left_obs = 0.0;
float quad_right_obs = 0.0;
float quad_front_obs = 0.0;

const float safe_regin = 1.5;
const float puzzle_width = 3.0;
const float distance_to_landing_area = 6.0;
const float distance_from_entrance_to_puzzle = 5.5;
const float door_width_length = 1.0;

const float weight = 25; //in N,gravity = 10
const float balance_thrust = 39.0 ;
const float laragest_force_output = 43.36; // in N,gravity = 10

bool tf_receive_flag = false;
bool laser_flag = false;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_sdk");
  ros::NodeHandle nh;


  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub      = nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);
  ros::Subscriber velocitySub = nh.subscribe("dji_sdk/velocity",10, &velocity_callback);
  ros::Subscriber heightAboveTakeoffSub = nh.subscribe("dji_sdk/height_above_takeoff",10 , &height_above_takeoff_callback);

  // Subscribe to message from guidance_node
  //ros::Subscriber guidance_imuSub     = my_node.subscribe("/guidance/imu", 1, &guidance_imu_callback);
  //ros::Subscriber guidance_velocitySub = my_node.subscribe("/guidance/velocity", 1, &guidance_velocity_callback);
  ros::Subscriber guidance_ultrasonicSub = nh.subscribe("/guidance/ultrasonic", 1, &guidance_ultrasonic_callback);
  ros::Subscriber guidance_positionSub = nh.subscribe("/guidance/position", 1, &guidance_position_callback);

  //Subscribe to ust20lx lidar
  ros::Subscriber ust20lxSub = nh.subscribe("/scan", 1, &ust20lx_callback);


  // Subscribe to message from cartographer_2D
  ros::Subscriber cartographer2DSub = nh.subscribe<tf2_msgs::TFMessage>("/tf", 1000, &cartographer2D_callback);

  // Publish the control signal
  ctrlVelYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);

  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference    = nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");


  PID PIDX,PIDY,PIDZ,PIDVx,PIDVy,PIDVz;

  parameter PIDparam;
  string paraaddress("/home/m210/A3pro/src/Onboard-SDK-ROS/dji_sdk_demo/yaml/param_pid");
  if(PIDparam.readParam(paraaddress.c_str()) == 0)
  {
    std::cout << "config file error!" << std::endl;
    return 0;
  }

  //Init the PID of the position and velocity
  PIDX.setPID(PIDparam.x_p,PIDparam.x_i,PIDparam.x_d,PIDparam.x_f);
  PIDY.setPID(PIDparam.y_p,PIDparam.y_i,PIDparam.y_d,PIDparam.y_f);
  PIDZ.setPID(PIDparam.z_p,PIDparam.z_i,PIDparam.z_d,PIDparam.z_f);
  PIDVx.setPID(PIDparam.vx_p,PIDparam.vx_i,PIDparam.vx_d,PIDparam.vx_f);
  PIDVy.setPID(PIDparam.vy_p,PIDparam.vy_i,PIDparam.vy_d,PIDparam.vy_f);
  PIDVz.setPID(PIDparam.vz_p,PIDparam.vz_i,PIDparam.vz_d,PIDparam.vz_f);

  //config the upper limit of the integration, the maximum of control value and the dead zone of error
  PIDX.set_sat(2,1.0,0.0);//postion
  PIDY.set_sat(2,5.0,0.0);//z thrust
  PIDZ.set_sat(2,0.1,0.0);//roll
  PIDVx.set_sat(2,0.051,0.0);//degree
  PIDVy.set_sat(2,1.0,0.0);//yaw
  PIDVz.set_sat(2,1.0,0.0);


  PIDX.start_intergrate_flag = 1;
  PIDY.start_intergrate_flag = 1;
  PIDZ.start_intergrate_flag = 1;
  PIDVx.start_intergrate_flag = 1;
  PIDVy.start_intergrate_flag = 1;
  PIDVz.start_intergrate_flag = 1;



  bool obtain_control_result = obtain_control();  
  bool takeoff_result;


  if (!set_local_position())
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }



  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("A3/N3 taking off!");
    takeoff_result = A3monitoredTakeoff();
    //takeoff_result = A3TakeoffWithoutGPS();
  }

  inital_atti = current_atti;

  //PIDZ_velocity_control( PIDZ, cruse_height );


  //PIDVx_Degree_control(PIDVx, 10);

  //PIDXY_Traj_velocity_control(PIDX,PIDVy,10,10);
  //PIDXY_Traj_velocity_control(PIDX,PIDVy,10,15);
  //PIDXY_Traj_velocity_control(PIDX,PIDVy,18,10);
  //PIDXY_Traj_velocity_control(PIDX,PIDVy,10,10);
  PIDXY_Traj_degree_control(PIDVx,PIDVy,1,1);
  PIDXY_Traj_degree_control(PIDVx,PIDVy,1.5,2.0);
  PIDXY_Traj_degree_control(PIDVx,PIDVy,2.0,2.5);
  PIDXY_Traj_degree_control(PIDVx,PIDVy,2.5,2.5);



  //PIDYaw_Velocity_control(PIDVy,45);

  //PIDZ_thrust_control(PIDX,PIDX,5);





/*

  while( !( tf_receive_flag && laser_flag ) )  //wait for the cartographer to be ready
  {
    ros::spinOnce();
  }

  PIDZ_velocity_control( PIDZ, cruse_height );

  enter_puzzle( PIDX );
//  ros::Duration(1.5).sleep();

  move_to_door( PIDY );
//  ros::Duration(1.5).sleep();

  int game_end_flag = 1;

  while( game_end_flag )
  {

    //if( height < 2.0 )
   // {
   //   PIDZ_velocity_control( PIDZ, cruse_height );
   // }


    game_end_flag = move_to_saferegine( PIDX );
//    ros::Duration(1.5).sleep();

    if( game_end_flag == 1 )
    {
      move_to_door( PIDY );
//      ros::Duration(1.5).sleep();
    }

    ros::spinOnce();
  }

//  takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_LAND);

  std::cout << "end" << std::endl;
  ros::spin();
  return 0;

*/
}



bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
}


bool A3TakeoffWithoutGPS()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = guidance_height;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If A3 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     guidance_height - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}


bool A3monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (flight_status != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         display_mode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(5)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(5)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (flight_status != DJISDK::FlightStatus::STATUS_IN_AIR &&
         (display_mode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
         ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(20)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (display_mode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || display_mode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(20)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( display_mode != DJISDK::DisplayMode::MODE_P_GPS || display_mode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}


bool M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
  {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1: If M100 is not in the air after 10 seconds, fail.
  while (ros::Time::now() - start_time < ros::Duration(10))
  {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
     current_gps.altitude - home_altitude < 1.0)
  {
    ROS_ERROR("Takeoff failed.");
    return false;
  }
  else
  {
    start_time = ros::Time::now();
    ROS_INFO("Successful takeoff!");
    ros::spinOnce();
  }

  return true;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}


bool set_control(float x, float y, float z, float yaw, uint8_t flag)
{
  sensor_msgs::Joy ControlSetpoint;
  ControlSetpoint.axes.push_back(x);
  ControlSetpoint.axes.push_back(y);
  ControlSetpoint.axes.push_back(z);
  ControlSetpoint.axes.push_back(yaw);
  ControlSetpoint.axes.push_back(flag);

  ctrlVelYawPub.publish(ControlSetpoint);

}



geometry_msgs::Point rotation_coordinate_inverse(geometry_msgs::Quaternion q, geometry_msgs::Point Body)
{
  Eigen::Vector3d Body_temp(Body.x, Body.y, Body.z);
  Eigen::Quaterniond q_temp = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  Eigen::Vector3d ENU_temp = q_temp.toRotationMatrix() * Body_temp;
  geometry_msgs::Point output_temp;
  output_temp.x = ENU_temp(0);
  output_temp.y = ENU_temp(1);
  output_temp.z = ENU_temp(2);

  return output_temp;
}

geometry_msgs::Point rotation_coordinate(geometry_msgs::Quaternion q, geometry_msgs::Point ENU)
{
  Eigen::Vector3d ENU_temp(ENU.x, ENU.y, ENU.z);
  Eigen::Vector3d Body_temp;
  Eigen::Quaterniond q_temp = Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  Body_temp = q_temp.toRotationMatrix().inverse() * ENU_temp;
  geometry_msgs::Point output_temp;
  output_temp.x = Body_temp(0);
  output_temp.y = Body_temp(1);
  output_temp.z = Body_temp(2);

  return output_temp;
}


//get the time interval in second
float get_time_interval(ros::Time begin_time)
{
  ros::Time time_now = ros::Time::now();
  float time_interval_second  = time_now.sec - begin_time.sec;
  float time_interval_nsecond = time_now.nsec / 1e9 - begin_time.nsec / 1e9;
  return (time_interval_second + time_interval_nsecond);
}


geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;

}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  geometry_msgs::Point pos_temp;
  pos_temp = msg->point;
  current_local_pos = rotation_coordinate(inital_atti,pos_temp);
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  current_vel = *msg;

}

void height_above_takeoff_callback(const std_msgs::Float32::ConstPtr& msg)
{
  height = msg->data;
}

void guidance_ultrasonic_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  guidance_height_past = guidance_height;
  guidance_ultrosonic = *msg;
  guidance_height = guidance_ultrosonic.ranges[0];
  if(guidance_height < 0.1)
  {
    guidance_height = guidance_height_past;
  }
}

void guidance_position_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  guidance_pos = *msg;
}


void cartographer2D_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    //make sure the /tf is from cartographer
    if (msg->transforms[0].header.frame_id == "map")
    {
        tf2_msgs::TFMessage laser = *msg;
        cartographer2D_pos.x = laser.transforms[0].transform.translation.x;
        cartographer2D_pos.y = laser.transforms[0].transform.translation.y;
        cartographer2D_pos.z = 0.0;

        cartographer2D_q.w = laser.transforms[0].transform.rotation.w;
        cartographer2D_q.x = laser.transforms[0].transform.rotation.x;
        cartographer2D_q.y = laser.transforms[0].transform.rotation.y;
        cartographer2D_q.z = laser.transforms[0].transform.rotation.z;
        //std::cout << " recevied tf is " << cartographer2D_pos.x << std::endl;
        tf_receive_flag = true;
    }
}


void ust20lx_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ust20lx = *msg;
  //std::cout << " front " << ust20lx.ranges[540] << "  right  " << ust20lx.ranges[180] << " right " << ust20lx.ranges[900] <<std::endl;
  quad_front_obs = ust20lx.ranges[540];
  quad_left_obs = ust20lx.ranges[900];
  quad_right_obs = ust20lx.ranges[180];
  laser_flag = true;

}

void PIDX_velocity_control(PID PIDinput, float pos_value)
{

  float error_x = pos_value - current_local_pos.x;
  ros::Time start_time = ros::Time::now();
  while(abs(error_x) > error_thousold)
  {
    error_x = pos_value - current_local_pos.x;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_x,time_inv);
    PIDinput.pid_output();
    set_control(PIDinput.Output,0.0, 0.0,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }

}

void PIDVx_Degree_control(PID PIDinputP, float pos_value)
{

  float error_x = pos_value - current_local_pos.x;
  ros::Time start_time = ros::Time::now();
  while(abs(error_x) > error_thousold)
  {
    error_x = pos_value - current_local_pos.x;
    float time_inv = get_time_interval(start_time);
    PIDinputP.add_error(error_x,time_inv);
    PIDinputP.pid_output();


    std::cout << PIDinputP.Output << "  " << error_x << "  " << current_local_pos.x <<std::endl;
    set_control(PIDinputP.Output, PIDinputP.Output , 0.0, 0.0,degree_control_flag);

    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }

}

void PIDY_velocity_control(PID PIDinput,float pos_value)
{
  float error_y = pos_value - current_local_pos.y;
  ros::Time start_time = ros::Time::now();

  while(abs(error_y) > error_thousold)
  {
    error_y = pos_value - current_local_pos.y;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_y,time_inv);
    PIDinput.pid_output();
    set_control(0.0, PIDinput.Output, 0.0,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }

}

void  PIDZ_velocity_control(PID PIDinput,float pos_value)
{
  float error_z = pos_value - height;
  ros::Time start_time = ros::Time::now();

  while(abs(error_z) > error_thousold)
  {
    error_z = pos_value - current_local_pos.z;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_z,time_inv);
    PIDinput.pid_output();
    set_control(0.0,0.0,PIDinput.Output,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }
}

void PIDX_velocity_control_guidance(PID PIDinput,float pos_value)
{
  float error_x = pos_value - guidance_pos.vector.x;
  ros::Time start_time = ros::Time::now();
  while(abs(error_x) > error_thousold)
  {
    error_x = pos_value - guidance_pos.vector.x;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_x,time_inv);
    PIDinput.pid_output();
    set_control(PIDinput.Output,0.0, 0.0,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }

}

void PIDY_velocity_control_guidance(PID PIDinput,float pos_value, float error_input, ros::Time time_now)
{
  while(abs(error_input) > error_thousold)
  {
    error_input = pos_value - guidance_pos.vector.y;
    float time_inv = get_time_interval(time_now);
    PIDinput.add_error(error_input,time_inv);
    PIDinput.pid_output();
    set_control(0.0, PIDinput.Output, 0.0,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }

}

void PIDZ_velocity_control_guidance(PID PIDinput,float pos_value)
{
  float error_z = pos_value - guidance_height;
  ros::Time start_time = ros::Time::now();
  while(abs(error_z) > error_thousold)
  {
    error_z = pos_value - guidance_height;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_z,time_inv);
    PIDinput.pid_output();
    set_control(0.0,0.0,PIDinput.Output,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }
}

void PIDX_velocity_control_cartographer2D(PID PIDinput,float pos_value)
{
  float error_cart_x = pos_value - cartographer2D_pos.x;
  ros::Time start_time = ros::Time::now();
  while(abs(error_cart_x) > error_thousold)
  {
    error_cart_x = pos_value - cartographer2D_pos.x;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_cart_x,time_inv);
    PIDinput.pid_output();
    set_control(PIDinput.Output,0.0, 0.0,0.0,velocity_control_flag);

    std::cout << "error_x  " << error_cart_x << std::endl;
    if( quad_front_obs < 0.5 )
    {
      set_control(0.0, 0.0, 0.0,0.0,velocity_control_flag);
      break;
    }
    ros::Duration(0.002).sleep();
    ros::spinOnce();
  }

}

void PIDY_velocity_control_cartographer2D(PID PIDinput,float pos_value)
{
  float error_cart_y = pos_value - cartographer2D_pos.y;
  ros::Time start_time = ros::Time::now();
  while(abs(error_cart_y) > error_thousold)
  {
    error_cart_y = pos_value - cartographer2D_pos.y;
    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_cart_y,time_inv);
    PIDinput.pid_output();
    if( quad_front_obs < 0.8 )
    {
        set_control(-0.15, PIDinput.Output, 0.0,0.0,velocity_control_flag);
    }
    set_control(0.0, PIDinput.Output, 0.0,0.0,velocity_control_flag);

    //std::cout << "left  " << quad_left_obs << "right  " << quad_right_obs << std::endl;
    std::cout << "error_y  " << error_cart_y << std::endl;
    if( ( quad_left_obs < 0.8) || ( quad_right_obs < 0.8 ) )
    {
      set_control(0.0, 0.0, 0.0,0.0,velocity_control_flag);
      break;
    }

    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }

}

void PIDXY_velocity_control_cartographer2D(PID PIDinput_x, PID PIDinput_y, float pos_value_x, float pos_value_y)
{
  float error_cart_x = pos_value_x - cartographer2D_pos.x;
  float error_cart_y = pos_value_y - cartographer2D_pos.y;
  ros::Time start_time = ros::Time::now();
  while( (abs(error_cart_x) > error_thousold) && (abs(error_cart_y) > error_thousold) )
  {
    error_cart_x = pos_value_x - cartographer2D_pos.x;
    error_cart_y = pos_value_y - cartographer2D_pos.y;
    float time_inv = get_time_interval(start_time);
    PIDinput_x.add_error(error_cart_x,time_inv);
    PIDinput_y.add_error(error_cart_y,time_inv);
    PIDinput_x.pid_output();
    PIDinput_y.pid_output();
    set_control( PIDinput_x.Output, PIDinput_y.Output, 0.0,0.0,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();
  }
}

void test_thrust_balance(float start, float end, float dt)
{
  ros::Duration(1).sleep();
  float z = current_local_pos.z;
  std::cout <<"current height " << z << std::endl;

  int n = (end - start) / dt;
  for(int i = 0; i <= n; i ++)
  {
    set_control(0, 0, start + dt * i, 0, thrust_control_flag);
    std::cout << "current thrust is " << start + dt * i << "% , current height is " <<  current_local_pos.z << std::endl;
    if(current_local_pos.z - z > 0.1)
    {
      std::cout << start + dt * i << " is around balance point!";
      break;
    }

    ros::Duration(1).sleep();
    ros::spinOnce();

  }
}

void PIDZ_thrust_control(PID PidinputP, PID PidinputV, float pos_value_z)
{
  float error_z = pos_value_z - current_local_pos.z;
  ros::Time start_time = ros::Time::now();
  while(abs(error_z) > error_thousold)
  {
    error_z = pos_value_z - current_local_pos.z;
    float time_inv = get_time_interval(start_time);
    PidinputP.add_error(error_z,time_inv);
    PidinputP.pid_output();

    float error_vz = PidinputP.Output - current_vel.vector.z;
    time_inv = get_time_interval(start_time);
    PidinputV.add_error(error_vz, time_inv);
    PidinputV.pid_output();

    //float output_f = PidinputV.Output + weight;
    //set_control(0.0,0.0,output_f / laragest_force_output,0.0,thrust_control_flag);
    //std::cout << output_f / laragest_force_output << " " << PidinputV.Output <<std::endl;

    set_control(0.0,0.0, PidinputV.Output + balance_thrust, 0.0,thrust_control_flag);
    std::cout << PidinputV.Output + balance_thrust << " " << error_z << " " << current_local_pos.z << std::endl;
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }
}

float yaw_error_dealt(float yaw_value)
{
  yaw2 = toEulerAngle(current_atti).z * rad2deg - toEulerAngle(inital_atti).z *rad2deg;
  if(yaw2 - yaw1 > 300)
  {
    yaw2 -=360;
    if(yaw2 <= -360)
      yaw2 += 360;
  }
  if(yaw2 - yaw1 < -300)
  {
    yaw2 += 360;
    if(yaw2 >= 360)
      yaw2 -= 360;
  }
  float error_yaw = yaw_value - yaw2;
  if(error_yaw > 180)
    error_yaw -= 360;
  if(error_yaw < -180)
    error_yaw += 360;
  return error_yaw;
}

void PIDYaw_Velocity_control(PID PIDinput,float yaw_value) // yaw_value range from 0 to 360
{
  float error_yaw = yaw_error_dealt(yaw_value);

  ros::Time start_time = ros::Time::now();

  while(abs(error_yaw) > error_thousold_yaw)
  {
    error_yaw = yaw_error_dealt(yaw_value);

    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_yaw,time_inv);
    PIDinput.pid_output();
    set_control(0.0,0.0,0.0,PIDinput.Output,velocity_control_flag);
    ros::Duration(0.002).sleep();
    ros::spinOnce();

  }
}

//calulate the yaw angle in body frame(yaw range from 0 to 360)
float yaw_value_dealt(float x,float y)
{
  float angle = atan(y / x) * rad2deg;
  if((x > 0)&&(y < 0))
    angle = 360 - abs(angle);
  if((x < 0)&&(y > 0))
    angle = 180 - abs(angle);
  if((x < 0)&&(y < 0))
    angle = 180 + angle;

  return angle;
}


void PIDXY_Traj_velocity_control(PID PIDinput, PID PIDinputYaw, float pos_value_x, float pos_value_y)
{
  float error_x = pos_value_x - current_local_pos.x;
  float error_y = pos_value_y - current_local_pos.y;
  float input_l = sqrt(error_x * error_x + error_y * error_y);
  float error_l = input_l;
  float sinl = error_x / error_l;
  float cosl = error_y / error_l;

  float yaw_value = yaw_value_dealt(error_x, error_y);

  float error_yaw = yaw_error_dealt(yaw_value);
  std::cout << yaw_value_dealt(error_x, error_y) << " " << (toEulerAngle(current_atti).z - toEulerAngle(inital_atti).z) * rad2deg << std::endl;

  ros::Time start_time = ros::Time::now();

  while(error_l > error_thousold)
  {
    error_x = pos_value_x - current_local_pos.x;
    error_y = pos_value_y - current_local_pos.y;
    error_l = sqrt(error_x * error_x + error_y * error_y);
    sinl = error_x / error_l;
    cosl = error_y / error_l;

    //yaw_value = yaw_value_dealt(error_x, error_y);
    error_yaw = yaw_error_dealt(yaw_value);

    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_l,time_inv);
    PIDinput.pid_output();

    PIDinputYaw.add_error(error_yaw,time_inv);
    PIDinputYaw.pid_output();

    geometry_msgs::Point world, body;
    world.x = PIDinput.Output * sinl;
    world.y = PIDinput.Output * cosl;
    world.z = height;
    body = rotation_coordinate( current_atti,rotation_coordinate_inverse(inital_atti,world));

    set_control( body.x, body.y, 0.0, PIDinputYaw.Output, velocity_control_flag);
    yaw1 = yaw2;

    ros::Duration(0.002).sleep();
    ros::spinOnce();
  }
}


void PIDXYYaw_velocity_control(PID PIDinput, PID PIDinputYaw, float pos_value_x, float pos_value_y, float yaw_value)
{
  float error_x = pos_value_x - current_local_pos.x;
  float error_y = pos_value_y - current_local_pos.y;
  float input_l = sqrt(error_x * error_x + error_y * error_y);
  float error_l = input_l;
  float sinl = error_x / error_l;
  float cosl = error_y / error_l;

  float error_yaw = yaw_error_dealt(yaw_value);
  std::cout << yaw_value_dealt(error_x, error_y) << " " << (toEulerAngle(current_atti).z - toEulerAngle(inital_atti).z) * rad2deg << std::endl;

  ros::Time start_time = ros::Time::now();

  while(error_l > error_thousold)
  {
    error_x = pos_value_x - current_local_pos.x;
    error_y = pos_value_y - current_local_pos.y;
    error_l = sqrt(error_x * error_x + error_y * error_y);
    sinl = error_x / error_l;
    cosl = error_y / error_l;

    error_yaw = yaw_error_dealt(yaw_value);

    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_l,time_inv);
    PIDinput.pid_output();

    PIDinputYaw.add_error(error_yaw,time_inv);
    PIDinputYaw.pid_output();

    geometry_msgs::Point world, body;
    world.x = PIDinput.Output * sinl;
    world.y = PIDinput.Output * cosl;
    world.z = height;
    body = rotation_coordinate( current_atti,rotation_coordinate_inverse(inital_atti,world));

    set_control( body.x, body.y, 0.0, PIDinputYaw.Output, velocity_control_flag);
    yaw1 = yaw2;

    ros::Duration(0.002).sleep();
    ros::spinOnce();
  }
}


void PIDXY_Traj_degree_control(PID PIDinput, PID PIDinputYaw, float pos_value_x, float pos_value_y)
{
  float error_x = pos_value_x - current_local_pos.x;
  float error_y = pos_value_y - current_local_pos.y;
  float input_l = sqrt(error_x * error_x + error_y * error_y);
  float error_l = input_l;
  float sinl = error_x / error_l;
  float cosl = error_y / error_l;

  float yaw_value = yaw_value_dealt(error_x, error_y);

  float error_yaw = yaw_error_dealt(yaw_value);
  std::cout << yaw_value_dealt(error_x, error_y) << " " << (toEulerAngle(current_atti).z - toEulerAngle(inital_atti).z) * rad2deg << std::endl;

  ros::Time start_time = ros::Time::now();

  while(error_l > error_thousold)
  {
    error_x = pos_value_x - current_local_pos.x;
    error_y = pos_value_y - current_local_pos.y;
    error_l = sqrt(error_x * error_x + error_y * error_y);
    sinl = error_x / error_l;
    cosl = error_y / error_l;

    //yaw_value = yaw_value_dealt(error_x, error_y);
    error_yaw = yaw_error_dealt(yaw_value);

    float time_inv = get_time_interval(start_time);
    PIDinput.add_error(error_l,time_inv);
    PIDinput.pid_output();

    PIDinputYaw.add_error(error_yaw,time_inv);
    PIDinputYaw.pid_output();

    geometry_msgs::Point world, body;
    world.x = PIDinput.Output * sinl;
    world.y = PIDinput.Output * cosl;
    world.z = height;
    body = rotation_coordinate( current_atti,rotation_coordinate_inverse(inital_atti,world));
    std::cout << -body.y << " " << body.x << std::endl;
    set_control( -body.y, body.x, 0.0, PIDinputYaw.Output, degree_control_flag);
    yaw1 = yaw2;

    ros::Duration(0.002).sleep();
    ros::spinOnce();
  }
}


int degree_to_number(float degree)
{
  int temp = int ( 1080 * ( degree + 45 ) / 270 );
  return temp;
}

float number_to_degree(int number)
{
  float degree = float (270 * number / 1080 - 45 );
  return degree;
}

float laser_degree_to_distance(float degree) //in degree
{
  float distance = 0.0;
  int temp = degree_to_number(degree);
  if( 0 < temp < 1080 )
    distance = ust20lx.ranges[ temp ];
  else
    distance = 30;
  return distance;
}

float find_entrance()
{
  float safe_regin_temp = quad_front_obs;
  float start_distance = sqrt( ( safe_regin_temp * safe_regin_temp ) + ( quad_right_obs * quad_right_obs) );
  float sin_start_degree = safe_regin_temp / start_distance;
  if( ( sin_start_degree > 1.0 ) || ( sin_start_degree < -1.0 ) )
  {
    std::cout << "There is no exit!" << std::endl;
    return false;
  }
  int start_angle = int ( abs( asin(sin_start_degree) ) * rad2deg );

  float end_distance = sqrt( ( safe_regin_temp * safe_regin_temp ) + ( quad_left_obs * quad_left_obs) );
  float sin_end_degree = safe_regin_temp / end_distance;
  if( ( sin_end_degree > 1.0 ) || ( sin_end_degree < -1.0 ) )
  {
    std::cout << "There is no exit!" << std::endl;
    return false;
  }
  int end_angle = int ( 180 - abs( asin(sin_end_degree) ) * rad2deg );

  std::cout << "start angle " << start_angle << std::endl;
  std::cout << "end angle " << end_angle << std::endl;

  float distance_temp = 0.0;
  float start_angle_temp = start_angle;
  float end_angle_temp = end_angle;
  float move_distance = 0.0;
  float door_width = 0.0;
  float start_distance_QJ = 0.0;
  float end_distance_QI = 0.0;


  for(int i = start_angle_temp; i <= end_angle_temp; i++)
  {
    distance_temp = laser_degree_to_distance( float (i) );

    if( ( distance_temp - safe_regin_temp / sin( float (i) * deg2rad ) ) > 0.4 )
    {
      start_angle_temp = float (i);
      break;
    }
  }
  for(int i = start_angle_temp; i <= end_angle_temp; i++)
  {
    distance_temp = laser_degree_to_distance( float (i) );
    if( ( distance_temp - safe_regin_temp / sin( float (i) * deg2rad ) ) < 0.4 )
    {
      end_angle_temp = float (i);
      break;
    }
  }

  if( start_angle_temp > 90.0 )
  {
    start_distance_QJ = safe_regin_temp * ( 1.0 / tan( ( 180.0 - start_angle_temp ) * deg2rad ) );
  }
  if( start_angle_temp == 90.0 )
  {
    start_distance_QJ = 0.0;
  }
  if( start_angle_temp < 90.0 )
  {
    start_distance_QJ = - safe_regin_temp * ( 1.0 / tan( ( start_angle_temp ) * deg2rad ) );
  }

  if( end_angle_temp > 90.0 )
  {
    end_distance_QI = safe_regin_temp * ( 1.0 / tan( ( 180.0 - end_angle_temp ) * deg2rad ) );
  }
  if( end_angle_temp == 90.0 )
  {
    end_distance_QI = 0.0;
  }
  if( end_angle_temp < 90.0 )
  {
    end_distance_QI = - safe_regin_temp * ( 1.0 / tan( ( end_angle_temp ) * deg2rad ) );
  }

  if( ( start_distance_QJ < 0.0 ) && ( end_distance_QI > 0.0 ) )
  {
    door_width = abs( end_distance_QI ) + abs( start_distance_QJ );
  }
  door_width = abs( end_distance_QI - start_distance_QJ );
/*
  while( door_width < door_width_length )
  {
    for(int i = end_angle_temp; i <= end_angle; i++)
    {
      distance_temp = laser_degree_to_distance( float (i) );

      if( ( distance_temp - safe_regin_temp / sin( float (i) * deg2rad ) ) > 0.4 )
      {
        start_angle_temp = float (i);
        break;
      }
    }
    for(int i = start_angle_temp; i <= end_angle; i++)
    {
      distance_temp = laser_degree_to_distance( float (i) );
      if( ( distance_temp - safe_regin_temp / sin( float (i) * deg2rad ) ) < 0.4 )
      {
        end_angle_temp = float (i);
        break;
      }
    }

    if( start_angle_temp > 90.0 )
    {
      start_distance_QJ = safe_regin_temp * ( 1.0 / tan( ( 180.0 - start_angle_temp ) * deg2rad ) );
    }
    if( start_angle_temp == 90.0 )
    {
      start_distance_QJ = 0.0;
    }
    if( start_angle_temp < 90.0 )
    {
      start_distance_QJ = - safe_regin_temp * ( 1.0 / tan( ( start_angle_temp ) * deg2rad ) );
    }

    if( end_angle_temp > 90.0 )
    {
      end_distance_QI = safe_regin_temp * ( 1.0 / tan( ( 180.0 - end_angle_temp ) * deg2rad ) );
    }
    if( end_angle_temp == 90.0 )
    {
      end_distance_QI = 0.0;
    }
    if( end_angle_temp < 90.0 )
    {
      end_distance_QI = - safe_regin_temp * ( 1.0 / tan( ( end_angle_temp ) * deg2rad ) );
    }

    if( ( start_distance_QJ < 0.0 ) && ( end_distance_QI > 0.0 ) )
    {
      door_width = abs( end_distance_QI ) + abs( start_distance_QJ );
    }
    door_width = abs( end_distance_QI - start_distance_QJ );

    std::cout << "  door start angle " << start_angle_temp<< "  door end angle " << end_angle_temp << " door width " << door_width << std::endl;
    //std::cout << "door width " << door_width << std::endl;
  }*/

  move_distance = 0.5 * ( start_distance_QJ + end_distance_QI );

  std::cout << "door start angle " << start_angle_temp << std::endl;
  std::cout << "door end angle " << end_angle_temp << std::endl;
  std::cout << "move distance is (left positive)  " << move_distance << std::endl;

  return move_distance;
}

bool move_to_saferegine(PID PID_input)
{
  float laser_pos_now_x = cartographer2D_pos.x;
  float distance_to_saferegin = quad_front_obs - safe_regin ;

  std::cout << "laser pos now x   " << laser_pos_now_x << std::endl;
  std::cout << "left   " << quad_left_obs << std::endl;
  std::cout << "right   " << quad_right_obs << std::endl;
  std::cout << "front   " << quad_front_obs << std::endl;
  std::cout << "distance_to_saferegin   " << distance_to_saferegin << std::endl;

  if( ( distance_to_saferegin > puzzle_width + puzzle_width  ) )
  {
    if( quad_front_obs < safe_regin - 0.5 )
    {
      PIDX_velocity_control_cartographer2D(PID_input, cartographer2D_pos.x + quad_front_obs - safe_regin );
    }
    PIDX_velocity_control_cartographer2D( PID_input, laser_pos_now_x + safe_regin + safe_regin );
    if( quad_left_obs + quad_right_obs > 15.0 )
    {
      std::cout << "pass through EXIT, end the game!" << std::endl;
      return 0;
    }
    else
    {
      std::cout << "find two or more doors! Attention! pass one door once" << std::endl;
    }
  }
  else
  {
    if( quad_front_obs < safe_regin - 0.5 )
    {
      PIDX_velocity_control_cartographer2D(PID_input, cartographer2D_pos.x + quad_front_obs - safe_regin );
    }
    PIDX_velocity_control_cartographer2D( PID_input, laser_pos_now_x + distance_to_saferegin);
    std::cout << "pass through the door" << std::endl;
    return 1;
  }
}

void move_to_door(PID PID_input)
{
  float laser_pos_now_y = cartographer2D_pos.y;
  std::cout << "laser_pos_now_y " << laser_pos_now_y << std::endl;
  float distance_to_door = find_entrance();
  float distance_check = find_entrance();

  PIDY_velocity_control_cartographer2D( PID_input, laser_pos_now_y + distance_to_door);
  if( abs( distance_check ) > 0.2 )
  {
    PIDY_velocity_control_cartographer2D( PID_input, laser_pos_now_y + distance_check);
  }
}

void enter_puzzle(PID PID_input)
{
  float laser_pos_now_x = cartographer2D_pos.x;

  std::cout << "laser pos now x   " << laser_pos_now_x << std::endl;
  std::cout << "front  " << quad_front_obs << std::endl;

  PIDX_velocity_control_cartographer2D( PID_input, laser_pos_now_x + distance_from_entrance_to_puzzle );
  std::cout << "pass through the entrance" << std::endl;
}


