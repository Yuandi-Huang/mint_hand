#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "finger_driver/finger_driver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;
using std::placeholders::_1;
using std::placeholders::_2;

// Control table address
#define ADDR_OPERATING_MODE   11
#define ADDR_MIN_POSITION     48
#define ADDR_MAX_POSITION     52
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_CURRENT     102
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_CURRENT  126
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_TEMP     146

// Protocol version
#define PROTOCOL_VERSION      2.0 

// Default setting
#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyUSB0"

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
       
        // Create services
        torque_enable_srv = this->create_service<finger_manipulation::srv::TorqueEnable>(
            "/enable_torque",
            std::bind(&finger_driver::torqueEnable, this, _1, _2));
            
        torque_disable_srv = this->create_service<finger_manipulation::srv::TorqueDisable>(
            "/disable_torque",
            std::bind(&finger_driver::torqueDisable, this, _1, _2));

        get_enabled_status_srv = this->create_service<finger_manipulation::srv::GetEnabledStatus>(
            "/get_enabled_status",
            std::bind(&finger_driver::getEnabledStatusCallback, this, _1, _2));

        get_position_srv = this->create_service<finger_manipulation::srv::GetPosition>(
            "/get_position",
            std::bind(&finger_driver::getPresentPositionCallback, this, _1, _2));

        get_position_limits_srv = this->create_service<finger_manipulation::srv::GetPositionLimits>(
            "/get_position_limits",
            std::bind(&finger_driver::getPositionLimitsCallback, this, _1, _2));

        get_current_srv = this->create_service<finger_manipulation::srv::GetCurrent>(
            "/get_current",
            std::bind(&finger_driver::getPresentCurrentCallback, this, _1, _2));

        get_temperature_srv = this->create_service<finger_manipulation::srv::GetTemperature>(
            "/get_temperature",
            std::bind(&finger_driver::getPresentTemperatureCallback, this, _1, _2));

        set_operating_mode_srv = this->create_service<finger_manipulation::srv::SetOperatingMode>(
            "/set_operating_mode",
            std::bind(&finger_driver::setOperatingModeCallback, this, _1, _2));

        // Create subscribers
        goal_position_sub = this->create_subscription<finger_manipulation::msg::GoalPosition>(
            "/goal_position", 10,
            std::bind(&finger_driver::goalPositionCallback, this, _1));

        goal_current_sub = this->create_subscription<finger_manipulation::msg::GoalCurrent>(
            "/goal_current", 10,
            std::bind(&finger_driver::goalCurrentCallback, this, _1));
    }

    ~finger_driver() {
        portHandler->closePort();
    }

private:
    rclcpp::Service<finger_manipulation::srv::TorqueEnable>::SharedPtr torque_enable_srv;
    rclcpp::Service<finger_manipulation::srv::TorqueDisable>::SharedPtr torque_disable_srv;
    rclcpp::Service<finger_manipulation::srv::GetEnabledStatus>::SharedPtr get_enabled_status_srv;

    rclcpp::Service<finger_manipulation::srv::GetCurrent>::SharedPtr get_current_srv;
    rclcpp::Service<finger_manipulation::srv::GetPosition>::SharedPtr get_position_srv;
    rclcpp::Service<finger_manipulation::srv::GetPositionLimits>::SharedPtr get_position_limits_srv;
    rclcpp::Service<finger_manipulation::srv::GetTemperature>::SharedPtr get_temperature_srv;
    rclcpp::Service<finger_manipulation::srv::SetOperatingMode>::SharedPtr set_operating_mode_srv;

    rclcpp::Subscription<finger_manipulation::msg::GoalPosition>::SharedPtr goal_position_sub;
    rclcpp::Subscription<finger_manipulation::msg::GoalCurrent>::SharedPtr goal_current_sub;

    PortHandler * portHandler;
    PacketHandler * packetHandler;

    void torqueEnable(
        const std::shared_ptr<finger_manipulation::srv::TorqueEnable::Request> request,
        std::shared_ptr<finger_manipulation::srv::TorqueEnable::Response> response) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, request->id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        
        (void) response;
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID %d -- Result: %d", request->id, dxl_error);
        }
    }

    void torqueDisable(
        const std::shared_ptr<finger_manipulation::srv::TorqueDisable::Request> request,
        std::shared_ptr<finger_manipulation::srv::TorqueDisable::Response> response) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, request->id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        
        (void) response;
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable torque for ID %d -- Result: %d", request->id, dxl_error);
        }
    }
    
    void getPresentCurrentCallback(
        const std::shared_ptr<finger_manipulation::srv::GetCurrent::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetCurrent::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int16_t current = 0;

        dxl_comm_result = packetHandler->read2ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_PRESENT_CURRENT, 
            (uint16_t *)&current, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "getCurrent : [ID:%d] -> [CURRENT:%d]", 
                request->id, current);
            response->current = current;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
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
            RCLCPP_INFO(this->get_logger(), "getPosition : [ID:%d] -> [POSITION:%d]", 
                request->id, position);
            response->position = position;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get position for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }

    void getPositionLimitsCallback(
        const std::shared_ptr<finger_manipulation::srv::GetPositionLimits::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetPositionLimits::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int32_t min = 0;
        int32_t max = 0;

        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_MIN_POSITION, 
            (uint32_t *)&min, &dxl_error);

        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_MAX_POSITION, 
            (uint32_t *)&max, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "getPositionLimits : [ID:%d] -> [MIN:%d], [MAX:%d]", 
                request->id, min, max);
            response->min_position = min;
            response->max_position = max;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get position limits for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
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
            RCLCPP_INFO(this->get_logger(), "getTemperature : [ID:%d] -> [TEMPERATURE:%d]", 
                request->id, temperature);
            response->temperature = temperature;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get temperature for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }

    void setOperatingModeCallback(
        const std::shared_ptr<finger_manipulation::srv::SetOperatingMode::Request> request,
        std::shared_ptr<finger_manipulation::srv::SetOperatingMode::Response> response) {
        
        int dxl_comm_result = COMM_TX_FAIL;
        uint8_t dxl_error = 0;
        if (static_cast<OperatingMode>(request->opmode) != OperatingMode::POSITION && 
            static_cast<OperatingMode>(request->opmode) != OperatingMode::CURRENT  && 
            static_cast<OperatingMode>(request->opmode) != OperatingMode::CURRENT_BASED_POSITION) {
            RCLCPP_ERROR(this->get_logger(), "setOperatingMode: unsupported operating mode");
            return;
        }

        // Check if torque is disabled
        int torque_enabled = -1;
        dxl_comm_result = packetHandler->read1ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_TORQUE_ENABLE, 
            (uint8_t *)&torque_enabled, &dxl_error);
        
        if (torque_enabled == 1) {
            RCLCPP_ERROR(this->get_logger(), "Cannot set operating mode! ID %d's torque is still enabled", request->id);
            return;
        }
        else if (torque_enabled == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for ID %d -- Result: %d", 
            request->id, dxl_comm_result);
            return;
        }

        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, request->id, ADDR_OPERATING_MODE, (uint8_t)request->opmode);

        (void) response;
        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Changed ID %d's opmode to %d", request->id, request->opmode);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set operating mode for ID %d -- Result: %d", 
            request->id, dxl_comm_result);
        }
    }
    
    void getEnabledStatusCallback(
        const std::shared_ptr<finger_manipulation::srv::GetEnabledStatus::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetEnabledStatus::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int8_t torque_enabled = 0;

        dxl_comm_result = packetHandler->read1ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_TORQUE_ENABLE, 
            (uint8_t *)&torque_enabled, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "enabledStatus : [ID:%d] -> [ENABLED:%d]", 
                request->id, torque_enabled);
            response->enabled = torque_enabled;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to get enabled status for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }

    void goalPositionCallback(const finger_manipulation::msg::GoalPosition::SharedPtr msg) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint32_t position = static_cast<unsigned int>(msg->position);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "setPosition : [ID:%d] [POSITION:%d]", 
                msg->id, msg->position);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set position for ID %d -- Result: %d", 
                msg->id, dxl_comm_result);
        }
    }
    
    void goalCurrentCallback(const finger_manipulation::msg::GoalCurrent::SharedPtr msg) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint16_t current = static_cast<unsigned int>(msg->current);

        dxl_comm_result = packetHandler->write2ByteTxRx(
            portHandler, (uint16_t)msg->id, ADDR_GOAL_POSITION, current, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "setCurrent : [ID:%d] [CURRENT:%d]", 
                msg->id, msg->current);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to set current for ID %d -- Result: %d", 
                msg->id, dxl_comm_result);
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
