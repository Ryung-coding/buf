#include "dynamixel_worker.hpp"

// using namespace std::chrono_literals;
using namespace dynamixel;

// Constructor: perform port and packet handler initialization here
DynamixelNode::DynamixelNode(const std::string &device_name): Node("dynamixel_node"),
    portHandler_(nullptr),
    packetHandler_(nullptr),
    groupSyncWrite_(nullptr),
    groupSyncRead_(nullptr),
    heartbeat_state_(0)
{
  // Initialize PortHandler and PacketHandler using provided device name and protocol version
  portHandler_ = PortHandler::getPortHandler(device_name.c_str());
  packetHandler_ = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port and set baudrate
  if (!portHandler_->openPort()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the port: %s", device_name.c_str());
    rclcpp::shutdown();
    exit(1);
  }
  if (!portHandler_->setBaudRate(BAUDRATE)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
    portHandler_->closePort();
    rclcpp::shutdown();
    exit(1);
  }
  
  // Initialize Dynamixel motors and create GroupSync objects
  if (!init_Dynamixel()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Dynamixel motors");
    portHandler_->closePort();
    rclcpp::shutdown();
    exit(1);
  }  

  // Create ROS2 subscriber for joint values
  joint_val_subscriber_ = this->create_subscription<dynamixel_interfaces::msg::JointVal>( "joint_val", 1, std::bind(&DynamixelNode::armchanger_callback, this, std::placeholders::_1));

  // Create ROS2 publishers for heartbeat and motor positions
  heartbeat_publisher_ = this->create_publisher<watchdog_interfaces::msg::NodeState>("dynamixel_state", 1);
  motor_position_publisher_ = this->create_publisher<dynamixel_interfaces::msg::JointVal>("motor_position", 1);

  // Create timers for heartbeat and reading motor positions
  heartbeat_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamixelNode::heartbeat_timer_callback, this));
  dynamixel_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DynamixelNode::Dynamixel_Write_Read, this));
}

void DynamixelNode::armchanger_callback(const dynamixel_interfaces::msg::JointVal::SharedPtr msg) {
  arm_des[0][0] = static_cast<int>(msg->a1_q[0] * rad2ppr_J1 + 2048.0);  // Arm 1
  arm_des[1][0] = static_cast<int>(msg->a2_q[0] * rad2ppr_J1 + 2048.0);  // Arm 2
  arm_des[2][0] = static_cast<int>(msg->a3_q[0] * rad2ppr_J1 + 2048.0);  // Arm 3
  arm_des[3][0] = static_cast<int>(msg->a4_q[0] * rad2ppr_J1 + 2048.0);  // Arm 4
    
  for (int i = 1; i < 5; ++i) {
    arm_des[0][i] = static_cast<int>(msg->a1_q[i] * rad2ppr + 2048.0);   // Arm 1
    arm_des[1][i] = static_cast<int>(msg->a2_q[i] * rad2ppr + 2048.0);   // Arm 2
    arm_des[2][i] = static_cast<int>(msg->a3_q[i] * rad2ppr + 2048.0);   // Arm 3
    arm_des[3][i] = static_cast<int>(msg->a4_q[i] * rad2ppr + 2048.0);   // Arm 4
  }
}

bool DynamixelNode::Dynamixel_Write_Read() {
  /*  Write  */
  groupSyncWrite_->clearParam();
  
  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      uint8_t param_goal_position[4] = {
        DXL_LOBYTE(DXL_LOWORD(arm_des[i][j])),
        DXL_HIBYTE(DXL_LOWORD(arm_des[i][j])),
        DXL_LOBYTE(DXL_HIWORD(arm_des[i][j])),
        DXL_HIBYTE(DXL_HIWORD(arm_des[i][j]))
      };
      if (!groupSyncWrite_->addParam(DXL_IDS[i][j], param_goal_position)) {
        return false;
      }
    }
  }

  /*  Read  */
  groupSyncRead_->clearParam();

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
      if (!groupSyncRead_->addParam(DXL_IDS[i][j])) {return false;}
    }
  }

  if (groupSyncRead_->txRxPacket() != COMM_SUCCESS){return false;}

  // Retrieve the present position for each motor
  for (size_t i = 0; i < ARM_NUM; ++i) {

    uint8_t id = DXL_IDS[i][0];
    if (groupSyncRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
      int ppr = groupSyncRead_->getData(id, ADDR_PRESENT_POSITION, 4);
      arm_mea[i][0] = static_cast<double>(ppr) * ppr2rad_J1;
    }
    else {return false;}

    for (size_t j = 1; j < 5; ++j) {
      uint8_t id = DXL_IDS[i][j];
      if (groupSyncRead_->isAvailable(id, ADDR_PRESENT_POSITION, 4)) {
        int ppr = groupSyncRead_->getData(id, ADDR_PRESENT_POSITION, 4);
        arm_mea[i][j] = static_cast<double>(ppr) * ppr2rad;
      }
      else {return false;}
    }
  }
  
  /*  Publish  */
  dynamixel_interfaces::msg::JointVal msg;
  for (size_t j = 0; j < 5; ++j) {
    msg.a1_q[j] = arm_mea[0][j];
    msg.a2_q[j] = arm_mea[1][j];
    msg.a3_q[j] = arm_mea[2][j];
    msg.a4_q[j] = arm_mea[3][j];
  }
  motor_position_publisher_->publish(msg);

  return (groupSyncWrite_->txPacket() == COMM_SUCCESS);
}

bool DynamixelNode::init_Dynamixel() {
  uint8_t dxl_error = 0;

  for (size_t i = 0; i < ARM_NUM; ++i) {
    for (size_t j = 0; j < 5; ++j) {
        uint8_t id = DXL_IDS[i][j];
        if (packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_OPERATING_MODE, 3, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to set operating mode for motor ID " << static_cast<int>(id) << std::endl;
        return false;
      }
      if (packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS){
        std::cerr << "Failed to enable torque for motor ID " << static_cast<int>(id) << std::endl;
        return false;
      }
    }
  }

  // Set PID gains
  change_position_gain(1, 100, 30, 0);
  change_velocity_gain(1, 100, 20);
  change_position_gain(2, 2500, 270, 0);
  change_velocity_gain(2, 2100, 220);

  groupSyncWrite_ = new GroupSyncWrite(portHandler_, packetHandler_, ADDR_GOAL_POSITION, 4);
  groupSyncRead_  = new GroupSyncRead(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, 4);
  return true;
}

void DynamixelNode::change_position_gain(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain, uint16_t d_gain) {
  uint8_t dxl_error = 0;
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_POSITION_P_GAIN, p_gain, &dxl_error);
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_POSITION_I_GAIN, i_gain, &dxl_error);
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_POSITION_D_GAIN, d_gain, &dxl_error);
}

void DynamixelNode::change_velocity_gain(uint8_t dxl_id, uint16_t p_gain, uint16_t i_gain) {
  uint8_t dxl_error = 0;
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_VELOCITY_P_GAIN, p_gain, &dxl_error);
  packetHandler_->write2ByteTxRx(portHandler_, dxl_id, ADDR_VELOCITY_I_GAIN, i_gain, &dxl_error);
}

void DynamixelNode::heartbeat_timer_callback() {
  heartbeat_state_++;
  watchdog_interfaces::msg::NodeState state_msg;
  state_msg.state = heartbeat_state_;
  heartbeat_publisher_->publish(state_msg);
}
  
DynamixelNode::~DynamixelNode() {
  if (groupSyncWrite_){delete groupSyncWrite_;}
  if (groupSyncRead_){delete groupSyncRead_;}
  if (portHandler_){portHandler_->closePort();}
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<DynamixelNode>(DEVICE_NAME);
    rclcpp::spin(node);
  }
  catch (const std::exception &e) {std::cerr << "Exception caught in main: " << e.what() << std::endl;}
  rclcpp::shutdown();
  return 0;
}