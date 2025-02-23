import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtGui import QIntValidator

import finger_manipulation.srv
import finger_manipulation.msg
from rqt_finger_manipulation.main_window import main_window

import sys
import asyncio

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
        self.temperature_client = self.create_client(finger_manipulation.srv.GetTemperature, '/get_temperature')
        self.get_motor_status_client = self.create_client(finger_manipulation.srv.GetTorqueEnabled, '/get_torque_enabled')
        self.set_motor_status_client = self.create_client(finger_manipulation.srv.SetTorqueEnabled, '/set_torque_enabled')
        self.position_publisher = self.create_publisher(finger_manipulation.msg.GoalPosition, '/goal_position', self.qos_profile)

        while not self.position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service...')

        self.current_request = finger_manipulation.srv.GetCurrent.Request()
        self.position_request = finger_manipulation.srv.GetPosition.Request()
        self.temperature_request = finger_manipulation.srv.GetTemperature.Request()
        self.get_motor_status_request = finger_manipulation.srv.GetTorqueEnabled.Request()
        self.set_motor_status_request = finger_manipulation.srv.SetTorqueEnabled.Request()
        self.position_message = finger_manipulation.msg.GoalPosition()
    
    async def getTemperature(self, id):
        """
        for all service caller functions:
            return a coroutine object to later run as a task

        :param id: motor ID to query
        :return: coroutine which initializes future
        """ 
        self.temperature_request.id = id
        self.temperature_client.call_async(self.temperature_request)

    async def getPosition(self, id):
        self.position_request.id = id
        self.position_client.call_async(self.position_request)
    
    async def getCurrent(self, id):
        self.current_request.id = id
        self.current_client.call_async(self.current_request)
    
    async def getMotorStatus(self, id):
        self.get_motor_status_request.id = id
        self.get_motor_status_client.call_async(self.get_motor_status_request)

    async def setMotorStatus(self, id, enabled):
        self.set_motor_status_request.id = id
        self.set_motor_status_request.enabled = enabled
        self.set_motor_status_client.call_async(self.set_motor_status_request)

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

    def __init__(self, context):
        super(RqtPlugin, self).__init__(context)
        if not rclpy.ok:
            rclpy.init()

        self.controlNode = ControlNode()
        self._widget = QMainWindow()
        self.ui = main_window()
        self.ui.setupUi(self._widget)
        context.add_widget(self._widget)
        self.motor_widgets = {} # Dictionary mapping motor IDs to UI elements
        
        # Bind functions to individual motor widgets
        for ID in range(1, NUM_MOTORS + 1):
            self.motor_widgets[ID] = {
                "scrollbar": getattr(self.ui, f"{motorIDs[ID]}_scrollbar"),                      # Scrollbar for position control
                "goalpos_textbox": getattr(self.ui, f"{motorIDs[ID]}_goalpos_textbox"),          # Textbox for goal position
                "encoder_textbox": getattr(self.ui, f"{motorIDs[ID]}_encoder_textbox"),          # Textbox for present encoder position
                "current_textbox": getattr(self.ui, f"{motorIDs[ID]}_current_textbox"),          # Textbox for present current
                "temperature_textbox": getattr(self.ui, f"{motorIDs[ID]}_temperature_textbox"),  # Textbox for present temperature
                "status_indicator": getattr(self.ui, f"{motorIDs[ID]}_enabled_label"),           # Label for motor torque enabled status
            }

            # Attach functions to each motor UI element
            self.motor_widgets[ID]["scrollbar"].valueChanged.connect(
                lambda value, ID = ID: self.setPositionScrollbar(
                    id = ID, 
                    position = self.convertEncoderPosition(value), 
                    textbox = self.motor_widgets[ID]["goalpos_textbox"]
                )
            )

            self.motor_widgets[ID]["goalpos_textbox"].setValidator(QIntValidator(0, 4096))

            self.motor_widgets[ID]["goalpos_textbox"].returnPressed.connect(
                lambda ID = ID: self.setPositionTextbox(
                    id = ID, 
                    position = self.motor_widgets[ID]["goalpos_textbox"].text(), 
                    scrollbar = self.motor_widgets[ID]["scrollbar"]
                )
            )

            self.motor_widgets[ID]["scrollbar"].valueChanged.connect(
                lambda value, ID = ID: self.setPositionScrollbar(
                    id = ID, 
                    position = self.convertEncoderPosition(value), 
                    textbox = self.motor_widgets[ID]["goalpos_textbox"]
                )
            )

        # Bind functions to global widgets
        self.ui.torque_enable_button.clicked.connect(lambda : asyncio.run(self.enable_all_motors()))
        self.ui.torque_disable_button.clicked.connect(lambda : asyncio.run(self.disable_all_motors()))

        # Start update loop
        self.timer = QTimer(self)
        self.timer.setSingleShot(False)
        self.timer.setInterval(100) # Display is updated every 100 ms
        self.timer.timeout.connect(lambda : asyncio.run(self.update())) 
        self.timer.start() 

    async def update(self): # Call position, current and temperature services to update display
        for ID in range(1, NUM_MOTORS + 1):
            position, temperature, current, status = await asyncio.gather(
                self.controlNode.getPosition(id=ID),
                self.controlNode.getTemperature(id=ID),
                self.controlNode.getCurrent(id=ID),
                self.controlNode.getMotorStatus(id=ID)
            )

            if status is None: # BLUE = Motor is not connected
                self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(16, 0, 188);")
            elif status == 1:  # GREEN = Motor torque enabled
                self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(1, 186, 53);")
            else:              # RED = Motor torque disabled
                self.motor_widgets[ID]["status_indicator"].setStyleSheet("background-color: rgb(255, 74, 74);")

            self.motor_widgets[ID]["encoder_textbox"].setText(str(position))
            self.motor_widgets[ID]["current_textbox"].setText(str(current))
            self.motor_widgets[ID]["temperature_textbox"].setText(str(temperature))
    
    async def enable_all_motors(self):
        for ID in range(1, NUM_MOTORS + 1):
            await self.controlNode.setMotorStatus(id = ID, enabled = True)

    async def disable_all_motors(self):
        for ID in range(1, NUM_MOTORS + 1):
            await self.controlNode.setMotorStatus(id = ID, enabled = False)

    # Convert to encoder value given a percentage of max value
    def convertEncoderPosition(self, percent):
        min = 0
        max = 4096
        return int((percent / 1000) * max + min)
    
    def setPositionTextbox(self, id, position, scrollbar):
        if not position: position = 0
        scrollbar.setValue(int(int(position) / 4096 * 1000))
        self.controlNode.setPosition(id = id, position = int(position))

    def setPositionScrollbar(self, id, position, textbox):
        if not position: position = 0
        textbox.setText(str(position))
        self.controlNode.setPosition(id = id, position = int(position))

def main():
    sys.exit(Main().main(sys.argv, standalone="rqt_finger_manipulation.rqt_finger_manipulation"))