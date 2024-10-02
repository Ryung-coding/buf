#ifndef CONTROLLER_MAIN
#define CONTROLLER_MAIN

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// PID Controller Gains
float Kp[5] = {0.0425, 14.10, 1.220, 0.100, 0.100};
float Kd[5] = {0.0108, 1.800, 0.006, 0.005, 0.005};
float Ki[5] = {0.0010, 0.000, 0.001,0*1.500, 0*1.500}; //1.5

// LQR Gains -> using MATLAB (Note* LQR solver code)
#define state_size 4
float LQR_K[4] = {0.000, 0.000, 0.000, 0.000};

// Control Constants
#define loop_hz 1000
#define sbus_center 1024
#define sbus_Range 676 
#define kill_bound 700
#define connect_bound 700
#define leg_up_bound 700
#define leg_down_bound 1400
#define top_leg_pos 2.3
#define mid_leg_pos 1.3
#define but_leg_pos 0.5
#define DEADZONE_SBUS 2
#define DEADZONE_INPUT 0
#define Lim_INPUT 4
#define anti_windup_gain 2

// Math Constants
#define deg2rad 0.01745329252
#define rad2deg 57.29577951
#define wheel_radius 0.0525 
#define RevPerS2RadPerS 6.283185307 

// Model Constants
#define CoM 0.22
#define pitch_offset 0.066

// Controller Values
float sbus_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Raw signal data
float ref[5] = {0.0, 0.0, 0.0, 0.0, 0.0};       // Filtered & transferred signal data { Heading Angle Velocity | Thrust Velocity | Leg Case | Web Connect  | Kill }
float I[5] = {0.0, 0.0, 0.0, 0.0, 0.0};         // Integral term
float ref_0_in=0;                               // yaw angle cmd Integral term
float ref_1_in=0;                               // position cmd Integral term
float ref_theta = 0.0;
float balancing_CMD = 0.0; 
float heading_CMD = 0.0;
bool controller_start = false;

// IMU Data
float imu_theta = 0.000444; 
float imu_theta_dot = 0.0; 
float imu_psi = 0.0; 
float imu_psi_dot = 0.0; 

// Motor Commands
float Motor_L_cmd = 0.0; 
float Motor_R_cmd = 0.0; 
float leg_CMD_L = 0.0; 
float leg_CMD_R = 0.0;

// Motor Sensing data
float pos_x=0.0;
float vel_x=0.0;
float leg_pos_0=0.0;
float leg_vel_0=0.0;
float leg_pos_1=0.0;
float leg_vel_1=0.0;

// State Flags
bool isKilled = true;
bool isConnected = true;

// GPS Data
float Long = 0.0;
float Lat = 0.0;
float Alt = 0.0;
float SIV = 0.0;
float FIX = 0.0;
float year = 0.0;
float month = 0.0;
float day = 0.0;
float hour = 0.0;
float minute = 0.0;
float second = 0.0;

// Timing
float dt = 1.0 / loop_hz;
rclcpp::Time last_time;


#endif // CONTROLLER_MAIN
