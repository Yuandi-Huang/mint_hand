#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "finger_driver/finger_driver.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;
using std::placeholders::_1;
using std::placeholders::_2;

// Control table addresses
#define ADDR_OPERATING_MODE   11
#define ADDR_MAX_POSITION     48
#define ADDR_MIN_POSITION     52
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_CURRENT     102
#define ADDR_GOAL_POSITION    116
#define ADDR_PRESENT_CURRENT  126
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_TEMP     146

// Protocol version
#define PROTOCOL_VERSION      2.0 

// Default setting
#define BAUDRATE              4000000
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
        
        // Create services
        set_torque_enabled_srv = this->create_service<finger_manipulation::srv::SetTorqueEnabled>(
            "/set_torque_enabled", std::bind(&finger_driver::setTorqueEnabledCallback, this, _1, _2));
        get_torque_enabled_srv = this->create_service<finger_manipulation::srv::GetTorqueEnabled>(
            "/get_torque_enabled", std::bind(&finger_driver::getTorqueEnabledCallback, this, _1, _2));
        get_position_srv = this->create_service<finger_manipulation::srv::GetPosition>(
            "/get_position", std::bind(&finger_driver::getPresentPositionCallback, this, _1, _2));
        get_position_bulk_srv = this->create_service<finger_manipulation::srv::GetPositionBulk>(
            "/get_position_bulk", std::bind(&finger_driver::getPresentPositionBulkCallback, this, _1, _2));
        get_position_limits_srv = this->create_service<finger_manipulation::srv::GetPositionLimits>(
            "/get_position_limits", std::bind(&finger_driver::getPositionLimitsCallback, this, _1, _2));
        set_position_limits_srv = this->create_service<finger_manipulation::srv::SetPositionLimits>(
            "/set_position_limits", std::bind(&finger_driver::setPositionLimitsCallback, this, _1, _2));
        get_current_srv = this->create_service<finger_manipulation::srv::GetCurrent>(
            "/get_current", std::bind(&finger_driver::getPresentCurrentCallback, this, _1, _2));
        get_current_bulk_srv = this->create_service<finger_manipulation::srv::GetCurrentBulk>(
            "/get_current_bulk", std::bind(&finger_driver::getPresentCurrentBulkCallback, this, _1, _2));
        get_temperature_srv = this->create_service<finger_manipulation::srv::GetTemperature>(
            "/get_temperature", std::bind(&finger_driver::getPresentTemperatureCallback, this, _1, _2));
        get_temperature_bulk_srv = this->create_service<finger_manipulation::srv::GetTemperatureBulk>(
            "/get_temperature_bulk", std::bind(&finger_driver::getPresentTemperatureBulkCallback, this, _1, _2));
        set_operating_mode_srv = this->create_service<finger_manipulation::srv::SetOperatingMode>(
            "/set_operating_mode", std::bind(&finger_driver::setOperatingModeCallback, this, _1, _2)); 

        // Create subscribers
        goal_position_sub = this->create_subscription<finger_manipulation::msg::GoalPosition>(
            "/goal_position", 10, std::bind(&finger_driver::goalPositionCallback, this, _1));
        goal_position_bulk_sub = this->create_subscription<finger_manipulation::msg::GoalPositionBulk>(
            "/goal_position_bulk", 10, std::bind(&finger_driver::goalPositionBulkCallback, this, _1));
        goal_current_sub = this->create_subscription<finger_manipulation::msg::GoalCurrent>(
            "/goal_current", 10, std::bind(&finger_driver::goalCurrentCallback, this, _1));
        goal_current_bulk_sub = this->create_subscription<finger_manipulation::msg::GoalCurrentBulk>(
            "/goal_current_bulk", 10, std::bind(&finger_driver::goalCurrentBulkCallback, this, _1));
    }

    ~finger_driver() {
        portHandler->closePort();
    }

private:
    rclcpp::Service<finger_manipulation::srv::SetTorqueEnabled>::SharedPtr set_torque_enabled_srv;
    rclcpp::Service<finger_manipulation::srv::GetTorqueEnabled>::SharedPtr get_torque_enabled_srv;

    rclcpp::Service<finger_manipulation::srv::GetCurrent>::SharedPtr get_current_srv;
    rclcpp::Service<finger_manipulation::srv::GetCurrentBulk>::SharedPtr get_current_bulk_srv;
    rclcpp::Service<finger_manipulation::srv::GetPosition>::SharedPtr get_position_srv;
    rclcpp::Service<finger_manipulation::srv::GetPositionBulk>::SharedPtr get_position_bulk_srv;
    rclcpp::Service<finger_manipulation::srv::GetPositionLimits>::SharedPtr get_position_limits_srv;
    rclcpp::Service<finger_manipulation::srv::SetPositionLimits>::SharedPtr set_position_limits_srv;
    rclcpp::Service<finger_manipulation::srv::GetTemperature>::SharedPtr get_temperature_srv;
    rclcpp::Service<finger_manipulation::srv::GetTemperatureBulk>::SharedPtr get_temperature_bulk_srv;
    rclcpp::Service<finger_manipulation::srv::SetOperatingMode>::SharedPtr set_operating_mode_srv;

    rclcpp::Subscription<finger_manipulation::msg::GoalPosition>::SharedPtr goal_position_sub;
    rclcpp::Subscription<finger_manipulation::msg::GoalPositionBulk>::SharedPtr goal_position_bulk_sub;
    rclcpp::Subscription<finger_manipulation::msg::GoalCurrent>::SharedPtr goal_current_sub;
    rclcpp::Subscription<finger_manipulation::msg::GoalCurrentBulk>::SharedPtr goal_current_bulk_sub;

    PortHandler * portHandler;
    PacketHandler * packetHandler;

    /**
     * @brief Handles requests for the set_torque_enabled service.
     *
     * This callback is invoked when a request to enable 
     * or disable torque is received for a specific motor ID.
     *
     * @param request Contains the motor ID and desired torque state (enabled/disabled).
     * @param response Unused.
     */
    void setTorqueEnabledCallback(
        const std::shared_ptr<finger_manipulation::srv::SetTorqueEnabled::Request> request,
        std::shared_ptr<finger_manipulation::srv::SetTorqueEnabled::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, request->id, ADDR_TORQUE_ENABLE, 
            (uint8_t)request->enabled, &dxl_error);
        
        (void) response;
        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                "setTorqueEnabled: Changed ID %d's enabled setting to %d", 
                request->id, request->enabled);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set enabled status for ID %d -- Result: %d", 
                request->id, dxl_error);
        }
    }
    
    /**
     * @brief Handles requests for the get_current service.
     *
     * This callback is invoked when a request for motor 
     * current in mA is received for a specific motor ID.
     *
     * @param request Contains the motor ID to query.
     * @param response Contains the present current in mA.
     */
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
            response->current = current;
            RCLCPP_INFO(this->get_logger(), 
                "getCurrent : [ID:%d] -> [CURRENT:%d]", 
                request->id, current);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to get current for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }
    
    /**
     * @brief Bulk read version of /get_current service
     *
     * @param request Contains an array of motor IDs.
     * @param response Contains an array of present current in mA.
     */
    void getPresentCurrentBulkCallback(
        const std::shared_ptr<finger_manipulation::srv::GetCurrentBulk::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetCurrentBulk::Response> response) {

        auto group_sync_read = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, 2);
        int num_motors = request->id.size();
        
        for (int i = 0; i < num_motors; i++)
            group_sync_read.addParam((uint8_t)request->id[i]);
        
        int dxl_comm_result = group_sync_read.txRxPacket();
        std::vector<int16_t> current(num_motors);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk read current -- Result: %d", dxl_comm_result);
            return;
        }

        for (int i = 0; i < num_motors; i++) {
            current[i] = group_sync_read.getData((uint8_t)request->id[i], ADDR_PRESENT_CURRENT, 2);
            RCLCPP_INFO(this->get_logger(), 
                "getCurrentBulk : [ID:%d] -> [CURRENT:%d]", 
                request->id[i], current[i]);
        }

        response->current = current;
    }

    /**
     * @brief Handles requests for the get_position service.
     *
     * This callback is invoked when a request for motor encoder 
     * position is received for a specific motor ID.
     *
     * @param request Contains the motor ID.
     * @param response Contains the present raw encoder position.
     */
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
            response->position = position;
            RCLCPP_INFO(this->get_logger(), 
                "getPosition : [ID:%d] -> [POSITION:%d]", 
                request->id, position);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to get position for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }
    
    /**
     * @brief Bulk read version of /get_position service
     *
     * @param request Contains an array of motor IDs.
     * @param response Contains an array of present raw encoder positions.
     */
    void getPresentPositionBulkCallback(
        const std::shared_ptr<finger_manipulation::srv::GetPositionBulk::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetPositionBulk::Response> response) {

        auto group_sync_read = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
        int num_motors = request->id.size();
        
        for (int i = 0; i < num_motors; i++)
            group_sync_read.addParam((uint8_t)request->id[i]);
        
        int dxl_comm_result = group_sync_read.txRxPacket();
        std::vector<int32_t> position(num_motors);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk read position -- Result: %d", dxl_comm_result);
            return;
        }

        for (int i = 0; i < num_motors; i++) {
            position[i] = group_sync_read.getData((uint8_t)request->id[i], ADDR_PRESENT_POSITION, 4);
            RCLCPP_INFO(this->get_logger(), 
                "getPositionBulk : [ID:%d] -> [POSITION:%d]", 
                request->id[i], position[i]);
        }

        response->position = position;
    }

    /**
     * @brief Handles requests for the get_position_limits service.
     *
     * This callback is invoked when a request for the range of motor
     * encoder position limits is received for a specific motor ID.
     *
     * @param request Contains the motor ID.
     * @param response Contains the min and max encoder position limits.
     */
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
            response->min_position = min;
            response->max_position = max;
            RCLCPP_INFO(this->get_logger(), 
                "getPositionLimits : [ID:%d] -> [MIN:%d], [MAX:%d]", 
                request->id, min, max);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to get position limits for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }

    /**
     * @brief Handles requests for the set_position_limits service.
     *
     * This callback is invoked when a request to set the range of motor
     * encoder position limits is received for a specific motor ID.
     *
     * @param request Contains the motor ID, plus min and max encoder position limits.
     * @param response Unused.
     */
    void setPositionLimitsCallback(
        const std::shared_ptr<finger_manipulation::srv::SetPositionLimits::Request> request,
        std::shared_ptr<finger_manipulation::srv::SetPositionLimits::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_MIN_POSITION, 
            (uint32_t)request->min_position, &dxl_error);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_MAX_POSITION, 
            (uint32_t)request->max_position, &dxl_error);

        (void) response;
        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                "setPositionLimits : [ID:%d] -> [MIN:%d], [MAX:%d]", 
                request->id, request->min_position, request->max_position);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set position limits for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }
    
    /**
     * @brief Handles requests for the get_temperature service.
     *
     * This callback is invoked when a request for motor temperature
     * in Celsius is received for a specific motor ID.
     *
     * @param request Contains the motor ID.
     * @param response Contains the present temperature in Celsius.
     */
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
            response->temperature = temperature;
            RCLCPP_INFO(this->get_logger(), 
                "getTemperature : [ID:%d] -> [TEMPERATURE:%d]", 
                request->id, temperature);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to get temperature for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }
    
    /**
     * @brief Bulk read version of /get_temperature service
     *
     * @param request Contains an array of motor IDs.
     * @param response Contains an array of the present temperatures in Celsius.
     */
    void getPresentTemperatureBulkCallback(
        const std::shared_ptr<finger_manipulation::srv::GetTemperatureBulk::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetTemperatureBulk::Response> response) {

        auto group_sync_read = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_TEMP, 1);
        int num_motors = request->id.size();
        
        for (int i = 0; i < num_motors; i++)
            group_sync_read.addParam((uint8_t)request->id[i]);
        
        int dxl_comm_result = group_sync_read.txRxPacket();
        std::vector<int8_t> temperature(num_motors);

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk read temperature -- Result: %d", dxl_comm_result);
            return;
        }

        for (int i = 0; i < num_motors; i++) {
            temperature[i] = group_sync_read.getData((uint8_t)request->id[i], ADDR_PRESENT_TEMP, 1);
            RCLCPP_INFO(this->get_logger(), 
                "getTemperatureBulk : [ID:%d] -> [TEMPERATURE:%d]", 
                request->id[i], temperature[i]);
        }

        response->temperature = temperature;
    }
    
    /**
     * @brief Handles requests for the set_operating_mode service.
     *
     * This callback is invoked when a request to set  
     * operating mode is received for a specific motor ID.
     *
     * @param request Contains the motor ID and desired operating mode:
     *                position, current, or current-based position.
     * @param response Unused.
     */
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
        uint8_t torque_enabled = 2;
        dxl_comm_result = packetHandler->read1ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_TORQUE_ENABLE, 
            (uint8_t *)&torque_enabled, &dxl_error);
        
        if (torque_enabled == 1) {
            RCLCPP_ERROR(this->get_logger(), 
                "Cannot set operating mode! ID %d's torque is still enabled", 
                request->id);
            return;
        }
        else if (torque_enabled == 2) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set operating mode for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
            return;
        }

        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, request->id, ADDR_OPERATING_MODE, (uint8_t)request->opmode);

        (void) response;
        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                "Changed ID %d's opmode to %d", 
                request->id, request->opmode);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set operating mode for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }
    
    /**
     * @brief Handles requests for the get_torque_enabled service.
     *
     * This callback is invoked when a request for a motor's torque
     * enabled status is received for a specific motor ID.
     *
     * @param request Contains the motor ID.
     * @param response Contains the motor's torque state (enabled/disabled).
     */
    void getTorqueEnabledCallback(
        const std::shared_ptr<finger_manipulation::srv::GetTorqueEnabled::Request> request,
        std::shared_ptr<finger_manipulation::srv::GetTorqueEnabled::Response> response) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        int8_t torque_enabled = 0;

        dxl_comm_result = packetHandler->read1ByteTxRx(
            portHandler, (uint8_t)request->id, ADDR_TORQUE_ENABLE, 
            (uint8_t *)&torque_enabled, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            response->enabled = torque_enabled;
            RCLCPP_INFO(this->get_logger(), 
                "getTorqueEnabled : [ID:%d] -> [ENABLED:%d]", 
                request->id, torque_enabled);
        } else {
            response->disconnected = true;
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to get enabled status for ID %d -- Result: %d", 
                request->id, dxl_comm_result);
        }
    }

    /**
     * @brief Callback function for the goal_position topic.
     *
     * This callback is invoked when a new message is received on the
     * goal_position topic for a specific motor ID. It processes the 
     * target encoder value and sends the movement command to the motor.
     *
     * @param msg Contains the motor ID and target encoder value.
     */
    void goalPositionCallback(const finger_manipulation::msg::GoalPosition::SharedPtr msg) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint32_t position = static_cast<unsigned int>(msg->position);

        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                "goalPosition : [ID:%d] [POSITION:%d]", 
                msg->id, msg->position);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set position for ID %d -- Result: %d", 
                msg->id, dxl_comm_result);
        }
    }
   
    /**
     * @brief Bulk publish version for goal_position topic.
     * @param msg Contains an array of motor IDs and goal positions.
     */
    void goalPositionBulkCallback(
        const finger_manipulation::msg::GoalPositionBulk::SharedPtr msg) {

        auto group_sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);
        int num_motors = (int)msg->id.size();
        if (num_motors != (int)msg->position.size()) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk write position -- invalid message");
            return;
        }
        
        for (int i = 0; i < num_motors; i++) {
            int32_t position_value = msg->position[i];
            uint8_t position_bytes[4];
            std::memcpy(position_bytes, &position_value, sizeof(int32_t));
            group_sync_write.addParam((uint8_t)msg->id[i], position_bytes);
        }

        int dxl_comm_result = group_sync_write.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk write goal position -- Result: %d", dxl_comm_result);
            return;
        }

        for (int i = 0; i < num_motors; i++) {
            RCLCPP_INFO(this->get_logger(), 
                "goalPositionBulk : [ID:%d] [POSITION:%d]", 
                msg->id[i], msg->position[i]);
        }
    }

    /**
     * @brief Callback function for the goal_current topic.
     *
     * This callback is invoked when a new message is received on the
     * goal_current topic for a specific motor ID. It processes the 
     * target current value in mA and sends the movement command to the motor.
     *
     * @param msg Contains the motor ID and target current in mA.
     */
    void goalCurrentCallback(const finger_manipulation::msg::GoalCurrent::SharedPtr msg) {

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint16_t current = static_cast<unsigned int>(msg->current);

        dxl_comm_result = packetHandler->write2ByteTxRx(
            portHandler, (uint16_t)msg->id, ADDR_GOAL_CURRENT, current, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), 
                "goalCurrent : [ID:%d] [CURRENT:%d]", 
                msg->id, msg->current);
        } else {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to set current for ID %d -- Result: %d", 
                msg->id, dxl_comm_result);
        }
    }

    /**
     * @brief Bulk publish version for goal_current topic.
     * @param msg Contains an array of motor IDs and goal currents.
     */
    void goalCurrentBulkCallback(
        const finger_manipulation::msg::GoalCurrentBulk::SharedPtr msg) {

        auto group_sync_write = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CURRENT, 2);
        int num_motors = (int)msg->id.size();
        if (num_motors != (int)msg->current.size()) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk write goal current -- invalid message");
            return;
        }
        
        for (int i = 0; i < num_motors; i++) {
            int16_t current_value = msg->current[i];
            uint8_t current_bytes[2];
            std::memcpy(current_bytes, &current_value, sizeof(int16_t));
            group_sync_write.addParam((uint8_t)msg->id[i], current_bytes);
        }

        int dxl_comm_result = group_sync_write.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to bulk write goal current -- Result: %d", dxl_comm_result);
            return;
        }

        for (int i = 0; i < num_motors; i++) {
            RCLCPP_INFO(this->get_logger(), 
                "goalCurrentBulk : [ID:%d] [CURRENT:%d]", 
                msg->id[i], msg->current[i]);
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
