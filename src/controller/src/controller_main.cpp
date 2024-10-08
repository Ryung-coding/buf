#include <controller_main.h>
#include <controller_main_function.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

using namespace
std::chrono_literals;
float ref_1_in=0;
float ref_0_in=0;

void balancing_controller()
{
        // velocity PID | Err_velocity -> desired theta
    ref_1_in+=ref[1]*dt*0.01;   
    ref_theta = computePID(ref_1_in, pos_x+0.22*imu_theta, vel_x+0.22*imu_theta_dot, dt, 0);
    
	    // LQR | u=-K*state
    // float state[state_size] = {imu_theta, imu_theta  _dot, pos_x, vel_x};
    // float desired_state[state_size] = {ref_theta, 0, 0, 0};
    // balancing_CMD = computeLQR(state, desired_state);
    balancing_CMD = computePID(ref_theta+0.066, imu_theta, imu_theta_dot, dt, 1);
	//balancing_CMD = 0;
}

void heading_controller()
{
        // Heading PID | Err_psi -> Err_psi_dot
    ref_0_in += ref[0]*0.01570796*dt;
    ref_0_in = fmod(ref_0_in + M_PI, 2.0 * M_PI);
    if (ref_0_in < 0) ref_0_in += 2.0 * M_PI;
    ref_0_in -= M_PI;
    heading_CMD = computePID(ref_0_in, imu_psi, imu_psi_dot, dt, 2);
    //heading_CMD = computePID(0, imu_psi, imu_psi_dot, dt, 2);
	//heading_CMD=0;
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Main_controller");

    auto sbus_subscription = node->create_subscription<sensor_msgs::msg::JointState>("sbus_data", 10, sbus_callback);
    auto sgps_subscription = node->create_subscription<sensor_msgs::msg::JointState>("gps_data", 10, gps_callback);
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::JointState>("imu_data", 10, imu_callback);
    auto dubal_subscription = node->create_subscription<sensor_msgs::msg::JointState>("dubal_data", 10, dubal_data_callback);

        // Left motor
    auto odrive_publisher_0 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint0_torque_controller/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_0;
    odrive_msg_0.data.resize(1);

        // Right motor
    auto odrive_publisher_1 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint1_torque_controller/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_1;
    odrive_msg_1.data.resize(1);

    last_time = node->now();
    rclcpp::WallRate loop_rate(loop_hz);
	
	//rclcpp::sleep_for(10s);
	//RCLCPP_INFO(rclcpp::get_logger("controller"), "GO!");
	//rclcpp::sleep_for(5s);
	//pos_x=0;
	//vel_x=0;

    while (rclcpp::ok())
    {
            // "dt" update
        rclcpp::Time current_time = node->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

            // controller
        balancing_controller();
        heading_controller();

        Motor_L_cmd = balancing_CMD - heading_CMD;
        Motor_R_cmd = balancing_CMD + heading_CMD;
	
			// rotation Setting
		Motor_L_cmd=-Motor_L_cmd;
		Motor_R_cmd=Motor_R_cmd;

        	// constrain
        if (abs(Motor_L_cmd) < DEADZONE_INPUT)  Motor_L_cmd = 0;
        if (abs(Motor_L_cmd) > Lim_INPUT)       Motor_L_cmd = Motor_L_cmd > 0 ? Lim_INPUT : -Lim_INPUT;
        if (abs(Motor_R_cmd) < DEADZONE_INPUT)  Motor_R_cmd = 0;
        if (abs(Motor_R_cmd) > Lim_INPUT)       Motor_R_cmd = Motor_R_cmd > 0 ? Lim_INPUT : -Lim_INPUT;

        if(isKilled==true)  odrive_msg_0.data[0] = 0;
        else                odrive_msg_0.data[0] = Motor_L_cmd;
        if(isKilled==true)  odrive_msg_1.data[0] = 0;
        else                odrive_msg_1.data[0] = Motor_R_cmd;        


        RCLCPP_INFO(rclcpp::get_logger("controller"), "pos:%f,%f | pitch:%f,%f | yaw:%f,%f", pos_x+0.22*imu_theta, ref_1_in, imu_theta, ref_theta+0.0066, imu_psi, ref_0_in); 
        //RCLCPP_INFO(rclcpp::get_logger("controller"), "%f|%f,%f|%f", imu_psi, ref_0_in,ref[0],heading_CMD); 
            // publisher
        odrive_publisher_0->publish(odrive_msg_0);
        odrive_publisher_1->publish(odrive_msg_1);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
        
    rclcpp::shutdown();
    return 0;
}
