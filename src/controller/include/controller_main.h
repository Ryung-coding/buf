#ifndef CONTROLLER_MAIN
#define CONTROLLER_MAIN

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// PID Controller Gains
float Kp[5] = {0.0205, 14.10, 1.5000, 0.050, 0.050};
float Kd[5] = {0.0088, 1.800, 0.0010, 0.000, 0.000};
float Ki[5] = {0.008, 0.000, 0.0001, 0.500, 0.500};


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
#define but_leg_ref 0.5
#define mid_leg_ref 1.3
#define top_leg_ref 2.3
#define DEADZONE_SBUS 0
#define DEADZONE_INPUT 0
#define Lim_INPUT 4
#define anti_windup_gain 2

// Math Constants
#define deg2rad 0.01745329252
#define rad2deg 57.29577951
#define wheel_radius 0.0525 
#define RevPerS2RadPerS 6.283185307 

//Model Constants
#define CoM 0.22
#define pitch_offset 0.066

// Controller Values
float sbus_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0}; // Raw signal data
float ref[5] = {0.0, 0.0, 0.0, 0.0, 0.0};       // Filtered & transferred signal data { Heading Angle Velocity | Thrust Velocity | Leg Case | Web Connect  | Kill }
float I[5] = {0.0, 0.0, 0.0, 0.0, 0.0};                   // Integral term
float ref_1_in = 0;
float ref_0_in = 0;

// IMU Data
float imu_theta = 0.000444; 
float imu_theta_dot = 0.000444; 
float imu_psi = 0.000444; 
float imu_psi_dot = 0.000444; 

// Motor Commands
float Motor_L_cmd = 0.0; 
float Motor_R_cmd = 0.0; 
float Leg_L_cmd = 0.0;
float Leg_R_cmd = 0.0;
float balancing_CMD = 0.0; 
float heading_CMD = 0.0;
float ref_theta = 0.0;
float ref_theta_dot = 0.0;

// Motor Sensing data
float pos_x = 0.0;
float vel_x = 0.0;
float odrive_leg_pos_0 = 0.0;
float odrive_leg_pos_1 = 0.0;
float odrive_leg_vel_0 = 0.0;
float odrive_leg_vel_1 = 0.0;

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
