#include <controller_main.h>
#include <controller_main_function.h>


void balancing_controller()
{
        // Position PID | Err_postion -> desired theta
    ref_theta = computePID(ref_1_in, pos_x+CoM*imu_theta, vel_x+CoM*imu_theta_dot, dt, 0);
        // Attitude PID | Err_theta -> input torque
    balancing_CMD = computePID(ref_theta+pitch_offset, imu_theta, imu_theta_dot, dt, 1);
}

void heading_controller()
{
        // Heading PID | Err_psi -> input torque
    heading_CMD = computePID(ref_0_in, imu_psi, imu_psi_dot, dt, 2);
}

void leg_controller()
{
        // leg_L PID | Err_pos -> Err_psi_dot
    Leg_L_cmd = computePID(ref[2], odrive_leg_pos_0, odrive_leg_vel_0, dt, 3);

        // leg_R PID | Err_psi -> Err_psi_dot
    Leg_R_cmd = computePID(-ref[2], odrive_leg_pos_1, odrive_leg_vel_1, dt, 4);
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

        // Left leg
    auto odrive_publisher_2 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint2_torque_controller/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_2;
    odrive_msg_2.data.resize(1);

        // Right leg
    auto odrive_publisher_3 = node->create_publisher<std_msgs::msg::Float64MultiArray>("/joint3_torque_controller/commands", 10);
    std_msgs::msg::Float64MultiArray odrive_msg_3;
    odrive_msg_3.data.resize(1);

    last_time = node->now();
    rclcpp::WallRate loop_rate(loop_hz);
	
    while (rclcpp::ok())
    {
            // "dt" update
        rclcpp::Time current_time = node->now();
        dt = (current_time - last_time).seconds();
        last_time = current_time;

        if(isKilled==true)  
        {
            Motor_L_cmd = 0;
            Motor_R_cmd = 0;
            I[0]=0.0;
            I[1]=0.0;
            I[2]=0.0;
            I[3]=0.0;
        }
        else
        {
                // controller
            balancing_controller();
            heading_controller();
            leg_controller();

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
        }                
     
        RCLCPP_INFO(rclcpp::get_logger("controller"), "torque: %f,%f | pos:%f,%f | pitch:%f,%f | yaw:%f,%f", Motor_L_cmd, Motor_R_cmd, pos_x+CoM*imu_theta, ref_1_in, imu_theta, ref_theta+pitch_offset, imu_psi, ref_0_in); 

        // RCLCPP_INFO(rclcpp::get_logger("controller"), "cmd : %f |sensor : %f,%f | toque : %f,%f", ref[2], odrive_leg_pos_0,odrive_leg_pos_1, Leg_L_cmd, Leg_R_cmd); 

        odrive_msg_0.data[0] = Motor_L_cmd;
        odrive_msg_1.data[0] = Motor_R_cmd;
        odrive_msg_2.data[0] = Leg_L_cmd;
        odrive_msg_3.data[0] = Leg_R_cmd;

        // odrive_msg_0.data[0] = 0;
        // odrive_msg_1.data[0] = 0;
        // odrive_msg_2.data[0] = 0;
        // odrive_msg_3.data[0] = 0;

        odrive_publisher_0->publish(odrive_msg_0);
        odrive_publisher_1->publish(odrive_msg_1);
        odrive_publisher_2->publish(odrive_msg_2);
        odrive_publisher_3->publish(odrive_msg_3);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
        
    rclcpp::shutdown();
    return 0;
}
