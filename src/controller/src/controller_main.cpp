#include <controller_main.h>
#include <controller_main_function.h>

void balancing_controller()
{
        // position PID | Err_position -> desired pitch(theta)
    ref_1_in+=0.01*ref[1]*dt;   
    ref_theta = computePID(ref_1_in, pos_x+CoM*imu_theta, vel_x+CoM*imu_theta_dot, dt, 0);
    
        // Attitude PID | Err pitch -> input torque
    balancing_CMD = computePID(ref_theta+pitch_offset, imu_theta, imu_theta_dot, dt, 1);

        // LQR | u=-K*state
    // float state[state_size] = {imu_theta, imu_theta  _dot, pos_x, vel_x};
    // float desired_state[state_size] = {ref_theta, 0, 0, 0};
    // balancing_CMD = computeLQR(state, desired_state);
}

void heading_controller()
{
        // Heading PID | Err_psi -> input delta torque
    ref_0_in += ref[0]*(M_PI/2.0)/100.0*dt;
    ref_0_in = fmod(ref_0_in + M_PI, 2.0*M_PI);
    if (ref_0_in < 0.0) ref_0_in += 2.0*M_PI;
    ref_0_in -= M_PI;

    heading_CMD = computePID(ref_0_in, imu_psi, imu_psi_dot, dt, 2);
}

void leg_controller()
{
        // leg PID | Err_leg_pos -> input torque
    leg_CMD_L = computePID(ref[2], leg_pos_0, leg_vel_0, dt, 3);
    leg_CMD_R = computePID(-ref[2], leg_pos_1, leg_vel_1, dt, 4);
	//leg_CMD_L = 0;
	//leg_CMD_R = 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Main_controller");

    auto sbus_subscription = node->create_subscription<sensor_msgs::msg::JointState>("sbus_data", 10, sbus_callback);
    auto sgps_subscription = node->create_subscription<sensor_msgs::msg::JointState>("gps_data", 10, gps_callback);
    auto imu_subscription = node->create_subscription<sensor_msgs::msg::JointState>("imu_data", 10, imu_callback);
    auto dubal_subscription = node->create_subscription<sensor_msgs::msg::JointState>("dubal_data", 10, dubal_data_callback);

        // wheel motor
    auto odrive_publisher_wheel = node->create_publisher<std_msgs::msg::Float64MultiArray>("/odrive_wheel/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_wheel;
    odrive_msg_wheel.data.resize(2);

        // leg motor
    auto odrive_publisher_leg = node->create_publisher<std_msgs::msg::Float64MultiArray>("/odrive_leg/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_leg;
    odrive_msg_leg.data.resize(2);

    last_time = node->now();
    rclcpp::WallRate loop_rate(loop_hz);

    while (rclcpp::ok())
    {
            // "dt" update
        rclcpp::Time current_time = node->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

            // controller
        if(controller_start==true)
		{
			balancing_controller();
        	heading_controller();
        	leg_controller();
		}

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

        if(isKilled==true)
        {
            odrive_msg_wheel.data[0] = 0;
            odrive_msg_wheel.data[1] = 0;
            odrive_msg_leg.data[0]= 0;
            odrive_msg_leg.data[1]= 0;
        }  
        else
        {
            odrive_msg_wheel.data[0] = Motor_L_cmd*0;
            odrive_msg_wheel.data[1] = Motor_R_cmd*0; 
            odrive_msg_leg.data[0]= leg_CMD_L;
            odrive_msg_leg.data[1]= leg_CMD_R;
        }                                     

        // RCLCPP_INFO(rclcpp::get_logger("controller"), "pos:%f,%f | pitch:%f,%f | yaw:%f,%f", pos_x+CoM*imu_theta, ref_1_in, imu_theta, ref_theta+pitch_offset, imu_psi, ref_0_in); 
        RCLCPP_INFO(rclcpp::get_logger("controller"), "leg:%f,%f|pos:%f||%f,%f|vel:%f,%f", leg_CMD_L, leg_CMD_R,ref[2],leg_pos_0,leg_pos_1,leg_vel_0,leg_vel_1); 
      	//odrive_msg_leg.data[0]=0;
		//odrive_msg_leg.data[1]=0;
            // publisher
        odrive_publisher_wheel->publish(odrive_msg_wheel);
        odrive_publisher_leg->publish(odrive_msg_leg);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
        
    rclcpp::shutdown();
    return 0;
}
