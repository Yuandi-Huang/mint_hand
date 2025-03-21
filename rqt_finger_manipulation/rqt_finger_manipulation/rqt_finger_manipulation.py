import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin
from PyQt5.QtCore import QTimer, QEventLoop
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtGui import QIntValidator

import finger_manipulation.srv
import finger_manipulation.msg
from rqt_finger_manipulation.main_window import main_window

import sys
import asyncio
import threading

NUM_MOTORS = 4

# Dictionary mapping motor IDs to joint names
motorIDs = {
    1 : "ABD",
    2 : "MCP",
    3 : "PIP",
    4 : "DIP"
}

# Reverse dictionary for joint names to motor IDs
motorNames = {v: k for k, v in motorIDs.items()}

class ControlNode(Node):
    def __init__(self):
        super().__init__('rqt_finger_manipulation')

        self.current_client = self.create_client(finger_manipulation.srv.GetCurrent, '/get_current')
        self.position_client = self.create_client(finger_manipulation.srv.GetPosition, '/get_position')
        self.get_position_limits_client = self.create_client(finger_manipulation.srv.GetPositionLimits, '/get_position_limits')
        self.set_position_limits_client = self.create_client(finger_manipulation.srv.SetPositionLimits, '/set_position_limits')
        self.temperature_client = self.create_client(finger_manipulation.srv.GetTemperature, '/get_temperature')
        self.get_motor_status_client = self.create_client(finger_manipulation.srv.GetTorqueEnabled, '/get_torque_enabled')
        self.set_motor_status_client = self.create_client(finger_manipulation.srv.SetTorqueEnabled, '/set_torque_enabled')
        self.position_publisher = self.create_publisher(finger_manipulation.msg.GoalPosition, '/goal_position', self.qos_profile)

        while not self.position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service...')

        self.current_request = finger_manipulation.srv.GetCurrent.Request()
        self.position_request = finger_manipulation.srv.GetPosition.Request()
        self.get_position_limits_request = finger_manipulation.srv.GetPositionLimits.Request()
        self.set_position_limits_request = finger_manipulation.srv.SetPositionLimits.Request()
        self.temperature_request = finger_manipulation.srv.GetTemperature.Request()
        self.get_motor_status_request = finger_manipulation.srv.GetTorqueEnabled.Request()
        self.set_motor_status_request = finger_manipulation.srv.SetTorqueEnabled.Request()
        self.position_message = finger_manipulation.msg.GoalPosition()
    
    async def getTemperature(self, id):
        """
        for all service caller functions:
            input a single ID
            generate a future using client.call_async for ID
            spin node until future is complete
            return future's result

        :param id: motor ID to query
        :return: value of result
        """ 
        self.temperature_request.id = id
        future = self.temperature_client.call_async(self.temperature_request)
        await future
        return future.result().temperature

    async def getPosition(self, id):
        self.position_request.id = id
        future = self.position_client.call_async(self.position_request)
        await future
        return future.result().position
    
    async def getPositionLimits(self, id):
        self.get_position_limits_request.id = id
        future = self.get_position_limits_client.call_async(self.get_position_limits_request)
        await future
        return future.result().min_position, future.result().max_position
    
    async def getCurrent(self, id):
        self.current_request.id = id
        future = self.current_client.call_async(self.current_request)
        await future
        return future.result().current 
    
    async def getMotorStatus(self, id):
        self.get_motor_status_request.id = id
        future = self.get_motor_status_client.call_async(self.get_motor_status_request)
        await future
        return None if future.result().disconnected else future.result().enabled

    async def setPositionLimits(self, id, min, max):
        self.set_position_limits_request.id = id
        self.set_position_limits_request.min_position = min
        self.set_position_limits_request.max_position = max
        future = self.set_position_limits_client.call_async(self.set_position_limits_request)
        await future
        
    async def setMotorStatus(self, id, enabled):
        self.set_motor_status_request.id = id
        self.set_motor_status_request.enabled = enabled
        future = self.set_motor_status_client.call_async(self.set_motor_status_request)
        await future

    def setPosition(self, id, position):
        self.position_message.id = id
        self.position_message.position = position
        return self.position_publisher.publish(self.position_message)
    
    qos_profile = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST, 
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE
    )

class RqtPlugin(Plugin):
    motor_widgets = dict()   # Dictionary mapping motor IDs to UI elements
    connected_motors = set() # Set of motors with connection
    encoder_limits = dict()  # Dictionary mapping motor IDs to encoder limits
    default_pose = dict()    # Map of motor IDs to saved encoder positions
    calibrating = {          # Map of motor ID to whether it is being calibrated
        i: False for i in range(1, NUM_MOTORS + 1)
    }

    def __init__(self, context):
        super(RqtPlugin, self).__init__(context)
        if not rclpy.ok:
            rclpy.init()

        self.controlNode = ControlNode()
        self._widget = QMainWindow()
        self.ui = main_window()
        self.ui.setupUi(self._widget)
        context.add_widget(self._widget)
        
        # Bind functions to individual motor widgets
        for ID in range(1, NUM_MOTORS + 1):
            self.motor_widgets[ID] = {
                "scrollbar": getattr(self.ui, f"{motorIDs[ID]}_scrollbar"),                      # Scrollbar for position control
                "goalpos_textbox": getattr(self.ui, f"{motorIDs[ID]}_goalpos_textbox"),          # Textbox for goal position
                "prompt_label": getattr(self.ui, f"{motorIDs[ID]}_goalpos_label"),               # Label with input prompt (default goal position)
                "encoder_textbox": getattr(self.ui, f"{motorIDs[ID]}_encoder_textbox"),          # Textbox for present encoder position
                "current_textbox": getattr(self.ui, f"{motorIDs[ID]}_current_textbox"),          # Textbox for present current
                "temperature_textbox": getattr(self.ui, f"{motorIDs[ID]}_temperature_textbox"),  # Textbox for present temperature
                "calibrate_button": getattr(self.ui, f"{motorIDs[ID]}_calibrate_button"),        # Button to enter position calibration mode
                "status_indicator": getattr(self.ui, f"{motorIDs[ID]}_enabled_label"),           # Label for motor torque enabled status
            }

            # Attach functions to each motor UI element
            self.motor_widgets[ID]["scrollbar"].valueChanged.connect(
                lambda value, ID = ID: self.setPositionScrollbar(
                    id = ID, position = value
                )
            )

            self.motor_widgets[ID]["goalpos_textbox"].returnPressed.connect(
                lambda ID = ID: self.setPositionTextbox(
                    id = ID, 
                    position = self.motor_widgets[ID]["goalpos_textbox"].text()
                )
            )

            self.motor_widgets[ID]["calibrate_button"].clicked.connect(
                lambda _, ID = ID: self.calibratePosition(id = ID)
            )

        # Bind functions to global widgets
        self.ui.set_pose_button.clicked.connect(lambda : asyncio.run(self.setPose()))
        self.ui.reset_pose_button.clicked.connect(lambda : asyncio.run(self.resetPose()))
        self.ui.torque_enable_button.clicked.connect(lambda : asyncio.run(self.enableAllMotors()))
        self.ui.torque_disable_button.clicked.connect(lambda : asyncio.run(self.disableAllMotors()))
        self.ui.reconnect_button.clicked.connect(lambda : asyncio.run(self.initializeStates()))

        # Spin controlNode in separate thread
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.controlNode)
        thread = threading.Thread(target=executor.spin, daemon=True)
        thread.start()

        # DEBUG : self.connected_motors = set(range(1, NUM_MOTORS + 1))
        asyncio.run(self.initializeStates())

        # Start update loop
        self.timer = QTimer(self)
        self.timer.setSingleShot(False)
        self.timer.setInterval(300) # Display is updated every 100 ms
        self.timer.timeout.connect(lambda : asyncio.run(self.update()))
        self.timer.start()

    async def update(self): # Call position, current and temperature services to update display
        tasks = [] # Array of service call tasks
        for ID in self.connected_motors:
            tasks.append(self.controlNode.getPosition(id=ID))
            tasks.append(self.controlNode.getTemperature(id=ID))
            tasks.append(self.controlNode.getCurrent(id=ID))
            tasks.append(self.controlNode.getMotorStatus(id=ID))

        results = await asyncio.gather(*tasks, return_exceptions=True)
        for ID in self.connected_motors:
            position, temperature, current, status = results[4*(ID-1):4*ID]
            
            # Update status indicator
            if status == False:         # RED = Motor torque disabled
                self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(255, 74, 74);")
                goal_position = max(self.encoder_limits[ID][0], min(position, self.encoder_limits[ID][1]))
                self.motor_widgets[ID]["goalpos_textbox"].setText(str(goal_position))
                self.motor_widgets[ID]["scrollbar"].setValue(goal_position)
            elif self.calibrating[ID]:  # YELLOW = in calibration
                self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(244, 248, 0);")
            else:                       # GREEN = Motor torque enabled
                self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(1, 186, 53);")

            # Warn if joint is out of calibrated range
            if not self.calibrating[ID] and (position < self.encoder_limits[ID][0] | position > self.encoder_limits[ID][1]):
                self.motor_widgets[ID]["encoder_textbox"].setStyleSheet("background-color: rgb(252, 255, 128);")
            else: self.motor_widgets[ID]["encoder_textbox"].setStyleSheet("")

            self.motor_widgets[ID]["encoder_textbox"].setText(str(position))
            self.motor_widgets[ID]["current_textbox"].setText(str(current))
            self.motor_widgets[ID]["temperature_textbox"].setText(str(temperature))
    
    async def initializeStates(self):     
        """
        Identify all connected motors
        Disable widgets for disconnected motors
        Set range and default position for scrollbars
        """
        self.connected_motors.clear()
        self.encoder_limits.clear()
        self.default_pose.clear()
        disconnected_motors = set()
        for ID in range(1, NUM_MOTORS + 1):
            # Check if motor has connection
            connected = await self.controlNode.getMotorStatus(id=ID)
            if connected is None: 
                disconnected_motors.add(ID)
                continue

            min_position, max_position = await self.controlNode.getPositionLimits(id=ID)
            starting_position = await self.controlNode.getPosition(id=ID)

            self.connected_motors.add(ID)
            self.encoder_limits[ID] = min_position, max_position
            self.default_pose[ID] = starting_position
            self.motor_widgets[ID]["scrollbar"].setRange(min_position, max_position)
            self.motor_widgets[ID]["scrollbar"].setValue(starting_position)
            self.motor_widgets[ID]["scrollbar"].setEnabled(True)
            self.motor_widgets[ID]["goalpos_textbox"].setText(str(starting_position))
            self.motor_widgets[ID]["goalpos_textbox"].setReadOnly(False)
            self.motor_widgets[ID]["calibrate_button"].setEnabled(True)

        for ID in disconnected_motors:
            # BLUE = motor not connected
            self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(16, 0, 188);")
            self.motor_widgets[ID]["scrollbar"].setRange(0, 0)
            self.motor_widgets[ID]["scrollbar"].setEnabled(False)
            self.motor_widgets[ID]["goalpos_textbox"].setText("D/C")
            self.motor_widgets[ID]["goalpos_textbox"].setReadOnly(True)
            self.motor_widgets[ID]["calibrate_button"].setEnabled(False)

    def calibratePosition(self, id):
        """
        Main function for calibration mode. Not available if motor is disconnected.
        """
        button = self.motor_widgets[id]["calibrate_button"]
        scrollbar = self.motor_widgets[id]["scrollbar"]
        textbox = self.motor_widgets[id]["goalpos_textbox"]

        def exitCalibration():
            button.setChecked(False)
            self.calibrating[id] = False
            self.motor_widgets[id]["prompt_label"].setText("Goal Position:")
            textbox.setValidator(None)
            asyncio.run(self.controlNode.setMotorStatus(id=id, enabled=False))
            asyncio.run(self.controlNode.setPositionLimits(
                id=id, min=self.encoder_limits[id][0], max=self.encoder_limits[id][1]
            ))
            asyncio.run(self.controlNode.setMotorStatus(id=id, enabled=True))
            textbox.returnPressed.connect(
                lambda: self.setPositionTextbox(
                    id = id, 
                    position = textbox.text()
                )
            )
            scrollbar.setRange(self.encoder_limits[id][0], self.encoder_limits[id][1])
            return
        
        # Exit calibration mode if button is unchecked
        if not button.isChecked(): 
            exitCalibration()
            return

        # Remove limits on motor positions
        asyncio.run(self.controlNode.setMotorStatus(id=id, enabled=False))
        asyncio.run(self.controlNode.setPositionLimits(id=id, min=0, max=4095))
        asyncio.run(self.controlNode.setMotorStatus(id=id, enabled=True))
        scrollbar.setRange(0, 4095)

        # Update status indicator
        self.calibrating[id] = True
        self.motor_widgets[id]["prompt_label"].setText("Enter MIN Range:")
        
        # Override goalpos_textbox behavior
        textbox.returnPressed.disconnect()
        textbox.setValidator(QIntValidator(0, 4095))
        delay = 0 # How long to wait for motor to stop moving and continue
        loop = QEventLoop()
        textbox.returnPressed.connect(
            lambda : self.setCalibrationTextbox(
                id = id, 
                position = int(textbox.text()),
                loop = loop, delay = delay
            )
        )
        loop.exec() # Halt execution until min range is set
        min = int(textbox.text())

        self.motor_widgets[id]["prompt_label"].setText("Enter MAX Range:")
        delay = 300 # Wait 300 ms to ensure motor reaches destination
        loop.exec() # Halt execution until max range is set
        max = int(textbox.text())

        # Update motor limits and widgets
        scrollbar.setRange(min, max)
        self.encoder_limits[id] = min, max

        exitCalibration()

    async def enableAllMotors(self):
        for ID in self.connected_motors:
            # Keep torque disabled if joint is out of calibrated range
            if (int(self.motor_widgets[ID]["encoder_textbox"].text()) < self.encoder_limits[ID][0] or
                int(self.motor_widgets[ID]["encoder_textbox"].text()) > self.encoder_limits[ID][1]):
                continue
            else: 
                await self.controlNode.setMotorStatus(id = ID, enabled = True)

    async def disableAllMotors(self):
        for ID in self.connected_motors:
            await self.controlNode.setMotorStatus(id = ID, enabled = False)
    
    async def setPose(self):
        for ID in self.connected_motors:
            position = await self.controlNode.getPosition(id=ID)
            self.default_pose[ID] = position

    async def resetPose(self):
        for ID in self.connected_motors:
            goal_position = self.default_pose[ID]
            self.controlNode.setPosition(id = ID, position = goal_position)
            self.motor_widgets[ID]["goalpos_textbox"].setText(str(goal_position))
            self.motor_widgets[ID]["scrollbar"].setValue(goal_position)

    def setPositionTextbox(self, id, position):
        try: # Ensure position is an int
            position = int(position)
            textbox = self.motor_widgets[id]["goalpos_textbox"]
            scrollbar = self.motor_widgets[id]["scrollbar"]
            if not position: position = scrollbar.minimum()

            if position < scrollbar.minimum(): position = scrollbar.minimum()
            if position > scrollbar.maximum(): position = scrollbar.maximum()
            
            textbox.setText(str(position))
            scrollbar.setValue(position)
            self.controlNode.setPosition(id = id, position = int(position))
        except: textbox.clear()

    def setPositionScrollbar(self, id, position):
        if not position: position = 0
        self.motor_widgets[id]["goalpos_textbox"].setText(str(position))
        self.controlNode.setPosition(id = id, position = int(position))
    
    def setCalibrationTextbox(self, id, position, loop, delay):
        """
        Variation of setPosition for while in calibration mode. 
        Delay is necessary as setting position limits requires 
        torque to be disabled. This ensures the motor reaches its destination.
        """
        textbox = self.motor_widgets[id]["goalpos_textbox"]
        scrollbar = self.motor_widgets[id]["scrollbar"]

        textbox.setText(str(position))
        scrollbar.setValue(position)
        self.controlNode.setPosition(id = id, position = int(position))

        # Allow delay for motor to finish moving
        QTimer.singleShot(delay, loop.quit)
    
    def shutdown_plugin(self):
        asyncio.run(self.disableAllMotors())

def main():
    sys.exit(Main().main(sys.argv, standalone="rqt_finger_manipulation.rqt_finger_manipulation"))