#include "dynamixel_worker.hpp"

using namespace std::chrono_literals;
using namespace dynamixel;

PortHandler *portHandler;
PacketHandler *packetHandler;
GroupSyncWrite *groupSyncWrite;
GroupSyncRead *groupSyncRead;

// std::array<int, TOTAL_MOTORS> DXL_DES_POS;
// std::array<int, TOTAL_MOTORS> DXL_CUR_POS;
// std::array<uint8_t, TOTAL_MOTORS> DXL_IDS;

// only use arm1
std::array<int, TOTAL_MOTORS> DXL_DES_POS = {};
std::array<int, TOTAL_MOTORS> DXL_CUR_POS = {};
std::array<uint8_t, TOTAL_MOTORS> DXL_IDS = {1, 2, 3, 4, 5};  // ID 1~5만 사용
// only use arm1


DynamixelNode::DynamixelNode() : Node("dynamixel_node") {
    // ROS2 Subscribers
    joint_val_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>("joint_val", 1, std::bind(&DynamixelNode::armchanger_callback, this, std::placeholders::_1));
    
    // ROS2 Publisher
    heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("dynamixel_state", 1);
    motor_position_publisher_ = this->create_publisher<MotorPositionMsg>("motor_position", 1);
    heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamixelNode::heartbeat_timer_callback, this));
    read_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&DynamixelNode::read_timer_callback, this));
}

void DynamixelNode::set_position_pid(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
    uint8_t dxl_error = 0;

    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_POSITION_P_GAIN, p_gain, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_POSITION_I_GAIN, i_gain, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_POSITION_D_GAIN, d_gain, &dxl_error);

}

void DynamixelNode::set_velocity_pid(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain) {
    uint8_t dxl_error = 0;

    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_VELOCITY_P_GAIN, p_gain, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, dxl_id, ADDR_VELOCITY_I_GAIN, i_gain, &dxl_error);

}

void DynamixelNode::armchanger_callback(const JointValMsg::SharedPtr msg) {
    for (int i = 0; i < 5; ++i) arm_1_rad[i] = msg->a1_q[i];
    // for (int i = 0; i < 5; ++i) arm_2_rad[i] = msg->a2_q[i];
    // for (int i = 0; i < 5; ++i) arm_3_rad[i] = msg->a3_q[i];
    // for (int i = 0; i < 5; ++i) arm_4_rad[i] = msg->a4_q[i];

    save_des_pos(arm_1_rad, arm_2_rad, arm_3_rad, arm_4_rad);
    Dynamixel_Write();
}

void DynamixelNode::save_des_pos(double* arm_1, double* arm_2, double* arm_3, double* arm_4) {
    for (int i = 0; i < 5; ++i)
        DXL_DES_POS[i] = static_cast<int>(arm_1[i] * ((i == 0) ? 6.25 : 1.0) * rad2ppr + 2048.0);
    // for (int i = 0; i < 5; ++i)
    //     DXL_DES_POS[5 + i] = static_cast<int>(arm_2[i] * ((i == 0) ? 6.25 : 1.0) * rad2ppr + 2048.0);
    // for (int i = 0; i < 5; ++i)
    //     DXL_DES_POS[10 + i] = static_cast<int>(arm_3[i] * ((i == 0) ? 6.25 : 1.0) * rad2ppr + 2048.0);
    // for (int i = 0; i < 5; ++i)
    //     DXL_DES_POS[15 + i] = static_cast<int>(arm_4[i] * ((i == 0) ? 6.25 : 1.0) * rad2ppr + 2048.0);
}


bool DynamixelNode::init_Dynamixel() {
    uint8_t dxl_error = 0;
    for (uint8_t id : DXL_IDS) {
        if (packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, 3, &dxl_error) != COMM_SUCCESS) {
            return false;
        }
        if (packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS) {
            return false;
        }
    }

    set_position_pid(1, 100, 30, 0);  // P=100, I=20, D=0
    set_velocity_pid(1, 100, 20);     // Vel P=1000, I=30

    set_position_pid(2, 2500, 270, 0);  // P=2500, I=270, D=0
    set_velocity_pid(2, 2100, 220); 

    return true;
}

bool DynamixelNode::Dynamixel_Write() {
    groupSyncWrite->clearParam();

    for (size_t i = 0; i < TOTAL_MOTORS; ++i) {
        uint8_t param_goal_position[4] = {
            DXL_LOBYTE(DXL_LOWORD(DXL_DES_POS[i])),
            DXL_HIBYTE(DXL_LOWORD(DXL_DES_POS[i])),
            DXL_LOBYTE(DXL_HIWORD(DXL_DES_POS[i])),
            DXL_HIBYTE(DXL_HIWORD(DXL_DES_POS[i]))
        };
        if (!groupSyncWrite->addParam(DXL_IDS[i], param_goal_position)) {
            return false;
        }
    }
    return (groupSyncWrite->txPacket() == COMM_SUCCESS);
}

bool DynamixelNode::Dynamixel_Read() {
    groupSyncRead->clearParam();  // Clean up existing IDs.

    for (uint8_t id : DXL_IDS) {
        if (!groupSyncRead->addParam(id)) {
            std::cerr << "Failed to add parameter for ID " << static_cast<int>(id) << std::endl;
            return false;
        }
    }

    if (groupSyncRead->txRxPacket() != COMM_SUCCESS) {
        std::cerr << "Failed to sync read!" << std::endl;
        return false;
    }

    MotorPositionMsg msg;

    for (size_t i = 0; i < TOTAL_MOTORS; ++i) {
        uint8_t id = DXL_IDS[i];
        if (groupSyncRead->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
            DXL_CUR_POS[i] = groupSyncRead->getData(id, ADDR_PRESENT_POSITION, 4);
        } else {
            std::cerr << "Failed to read position for ID " << int(id) << std::endl;
            return false;
        }
    }

    for (size_t i = 0; i < 5; ++i) {
        msg.a1_q[i] = DXL_CUR_POS[i] * ppr2rad;
        // msg.a2_cur_pos[i] = DXL_CUR_POS[5 + i] * ppr2rad;
        // msg.a3_cur_pos[i] = DXL_CUR_POS[10 + i] * ppr2rad;
        // msg.a4_cur_pos[i] = DXL_CUR_POS[15 + i] * ppr2rad;

        msg.a1_q[i] = DXL_DES_POS[i] * ppr2rad;
        // msg.a2_des_pos[i] = DXL_DES_POS[5 + i] * ppr2rad;
        // msg.a3_des_pos[i] = DXL_DES_POS[10 + i] * ppr2rad;
        // msg.a4_des_pos[i] = DXL_DES_POS[15 + i] * ppr2rad;
    }

    motor_position_publisher_->publish(msg);
    return true;
}

void DynamixelNode::read_timer_callback() {
    Dynamixel_Read();
}

void DynamixelNode::heartbeat_timer_callback() {
    heartbeat_state_++;
    watchdog_interfaces::msg::NodeState state_msg;
    state_msg.state = heartbeat_state_;
    heartbeat_publisher_->publish(state_msg);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamixelNode>();

    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
      std::cerr << "Failed to open the port!" << std::endl;
      return -1;
    }
    if (!portHandler->setBaudRate(BAUDRATE)) {
        std::cerr << "Failed to set the baudrate!" << std::endl;
        portHandler->closePort();
        return -1;
    }

    if (!node->init_Dynamixel()) {
        portHandler->closePort();
        return -1;
    }
    
    groupSyncWrite = new GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
    groupSyncRead = new GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);

    rclcpp::spin(node);

    portHandler->closePort();
    delete groupSyncWrite;
    delete groupSyncRead;
    rclcpp::shutdown();
    return 0;
}