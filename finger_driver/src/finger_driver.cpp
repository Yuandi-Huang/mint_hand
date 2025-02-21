#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "finger_manipulation/srv/get_position.hpp"
#include "finger_manipulation/srv/get_temperature.hpp"
#include "finger_manipulation/srv/set_operating_mode.hpp"
#include "finger_manipulation/msg/set_position.hpp"
#include "finger_manipulation/msg/set_current.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;
using std::placeholders::_1;
using std::placeholders::_2;

// Control table address
#define ADDR_OPERATING_MODE   11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_CURRENT     102
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_TEMP     146

// Protocol version
#define PROTOCOL_VERSION      2.0 

// Default setting
#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyUSB0"

enum class FingerID : uint8_t {
    DIP = 1,
    PIP = 2,
    MCP = 3,
    ABD = 4
};

enum class OperatingMode : uint8_t {
    POSITION = 3,
    CURRENT = 0,
    CURRENT_BASED_POSITION = 5
};

class finger_driver : public rclcpp::Node {
public:
    finger_driver() : Node("finger_driver") {

        portHandler = PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        // if (!portHandler->openPort()) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
        //     rclcpp::shutdown();
        //     return;
        // }

        // if (!portHandler->setBaudRate(BAUDRATE)) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
        //     rclcpp::shutdown();
        //     return;
        // }
       
        connected = true;

        // Enable torque
        enableTorque(static_cast<uint8_t>(FingerID::DIP));
        enableTorque(static_cast<uint8_t>(FingerID::PIP));
        enableTorque(static_cast<uint8_t>(FingerID::MCP));
        enableTorque(static_cast<uint8_t>(FingerID::ABD));

        // Create services
        get_position_srv_ = this->create_service<finger_manipulation::srv::GetPosition>(
            "/get_position",
            std::bind(&finger_driver::getPresentPositionCallback, this, _1, _2));

        get_temperature_srv_ = this->create_service<finger_manipulation::srv::GetTemperature>(
            "/get_temperature",
            std::bind(&finger_driver::getPresentTemperatureCallback, this, _1, _2));

        set_operating_mode_srv_ = this->create_service<finger_manipulation::srv::SetOperatingMode>(
            "/set_operating_mode",
            std::bind(&finger_driver::setOperatingModeCallback, this, _1, _2));
        
        // Create subscribers
        set_position_sub_ = this->create_subscription<finger_manipulation::msg::SetPosition>(
            "/set_position", 10,
            std::bind(&finger_driver::setPositionCallback, this, _1));

        set_current_sub_ = this->create_subscription<finger_manipulation::msg::SetCurrent>(
            "/set_current", 10,
            std::bind(&finger_driver::setCurrentCallback, this, _1));
    }

    ~finger_driver() {
        portHandler->closePort();
    }

private:
    rclcpp::Service<finger_manipulation::srv::GetPosition>::SharedPtr get_position_srv_;
    rclcpp::Service<finger_manipulation::srv::GetTemperature>::SharedPtr get_temperature_srv_;
    rclcpp::Service<finger_manipulation::srv::SetOperatingMode>::SharedPtr set_operating_mode_srv_;

    rclcpp::Subscription<finger_manipulation::msg::SetPosition>::SharedPtr set_position_sub_;
    rclcpp::Subscription<finger_manipulation::msg::SetCurrent>::SharedPtr set_current_sub_;

    PortHandler * portHandler;
    PacketHandler * packetHandler;
    bool connected = false;
    OperatingMode operatingMode = OperatingMode::POSITION;

    void enableTorque(uint8_t id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d -- Result: %d", id, dxl_error);
        }
    }

    void disableTorque(uint8_t id) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable torque for ID %d -- Result: %d", id, dxl_error);
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

    void getPresentTemperatureCallback(
        const std::shared_ptr<finger_manipulation::srv::GetTemperature::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetTemperature::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int8_t temperature = 0;

        dxl_comm_result = packetHandler->read1ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_PRESENT_TEMP, 
            (uint8_t *)&temperature, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "getTemperature : [ID:%d] -> [TEMPERATURE:%d]", request->id, temperature);
            response->temperature = temperature;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get temperature! Result: %d", dxl_comm_result);
        }
    }

    void setOperatingModeCallback(
        const std::shared_ptr<finger_manipulation::srv::SetOperatingMode::Request> request,
        std::shared_ptr<finger_manipulation::srv::SetOperatingMode::Response> response) {
        
        int dxl_comm_result = COMM_TX_FAIL;
        if (static_cast<OperatingMode>(request->opmode) != OperatingMode::POSITION && 
            static_cast<OperatingMode>(request->opmode) != OperatingMode::CURRENT  && 
            static_cast<OperatingMode>(request->opmode) != OperatingMode::CURRENT_BASED_POSITION) {
            RCLCPP_ERROR(this->get_logger(), "setOperatingMode: unsupported operating mode");
            return;
        }

        for (uint8_t ID = 1; ID <= 4; ID++) {
            disableTorque(ID);
            dxl_comm_result = packetHandler->write1ByteTxRx(
                portHandler, ID, ADDR_OPERATING_MODE, (uint8_t)request->opmode);
            enableTorque(ID);
        }

        (void) response;
        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Success!");
            operatingMode = static_cast<OperatingMode>(request->opmode);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode! Result: %d", dxl_comm_result);
        }
    }

    void setPositionCallback(const finger_manipulation::msg::SetPosition::SharedPtr msg) {
        if (operatingMode != OperatingMode::POSITION &&
            operatingMode != OperatingMode::CURRENT_BASED_POSITION) {
            RCLCPP_ERROR(this->get_logger(), "Incorrect operating mode!");
            return;
        }

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
    
    void setCurrentCallback(const finger_manipulation::msg::SetCurrent::SharedPtr msg) {
        if (operatingMode != OperatingMode::CURRENT &&
            operatingMode != OperatingMode::CURRENT_BASED_POSITION) {
            RCLCPP_ERROR(this->get_logger(), "Incorrect operating mode!");
            return;
        }

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint16_t current = static_cast<unsigned int>(msg->current);

        dxl_comm_result = packetHandler->write2ByteTxRx(
            portHandler, (uint16_t)msg->id, ADDR_GOAL_POSITION, current, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "setCurrent : [ID:%d] [CURRENT:%d]", msg->id, msg->current);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set current! Result: %d", dxl_comm_result);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<finger_driver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
