#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "finger_manipulation/srv/get_position.hpp"
#include "finger_manipulation/msg/set_position.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;
using std::placeholders::_1;
using std::placeholders::_2;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION      2.0 

// Default setting
#define DXL1_ID               1
#define DXL2_ID               2
#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyUSB0"

class ReadWriteNode : public rclcpp::Node {
public:
    ReadWriteNode() : Node("read_write_node") {
      
        portHandler = PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
            rclcpp::shutdown();
            return;
        }

        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
            rclcpp::shutdown();
            return;
        }

        // Enable torque for both motors
        enableTorque(DXL1_ID);
        enableTorque(DXL2_ID);

        // Create ROS 2 service
        get_position_srv_ = this->create_service<finger_manipulation::srv::GetPosition>(
            "/get_position",
            std::bind(&ReadWriteNode::getPresentPositionCallback, this, _1, _2));

        // Create ROS 2 subscriber
        set_position_sub_ = this->create_subscription<finger_manipulation::msg::SetPosition>(
            "/set_position", 10,
            std::bind(&ReadWriteNode::setPositionCallback, this, _1));
    }

    ~ReadWriteNode() {
        portHandler->closePort();
    }

private:
    rclcpp::Service<finger_manipulation::srv::GetPosition>::SharedPtr get_position_srv_;
    rclcpp::Subscription<finger_manipulation::msg::SetPosition>::SharedPtr set_position_sub_;

    PortHandler * portHandler;
    PacketHandler * packetHandler;

    void enableTorque(uint8_t id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for Dynamixel ID %d", id);
        }
    }

    void getPresentPositionCallback(
        const std::shared_ptr<finger_manipulation::srv::GetPosition::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetPosition::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int32_t position = 0;

        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_PRESENT_POSITION, 
            (uint32_t *)&position, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "getPosition : [ID:%d] -> [POSITION:%d]", request->id, position);
            response->position = position;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get position! Result: %d", dxl_comm_result);
        }
    }

    void setPositionCallback(const finger_manipulation::msg::SetPosition::SharedPtr msg) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint32_t position = static_cast<unsigned int>(msg->position);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set position! Result: %d", dxl_comm_result);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReadWriteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
