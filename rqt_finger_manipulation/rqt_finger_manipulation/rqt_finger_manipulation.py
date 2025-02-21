import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rqt_gui.main import Main
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtGui import QIntValidator

import finger_manipulation.srv
import finger_manipulation.msg
from rqt_finger_manipulation.main_window import main_window
import sys

class ControlNode(Node):
    def __init__(self):
        super().__init__('rqt_finger_manipulation')
        self.temperature_client = self.create_client(finger_manipulation.srv.GetTemperature, '/get_temperature')
        self.position_client = self.create_client(finger_manipulation.srv.GetPosition, '/get_position')
        self.position_publisher = self.create_publisher(finger_manipulation.msg.SetPosition, '/set_position', self.qos_profile)

        while not self.position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.temperature_request = finger_manipulation.srv.GetTemperature.Request()
        self.position_request = finger_manipulation.srv.GetPosition.Request()
        self.position_message = finger_manipulation.msg.SetPosition()

    def getTemperature(self, id):
        self.temperature_request.id = id
        return self.temperature_client.call_async(self.temperature_request)

    def getPosition(self, id):
        self.position_request.id = id
        return self.position_client.call_async(self.position_request)
    
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

        self.ui.DIP_scrollbar.valueChanged.connect(lambda value: self.setPosition(id = 4, position = self.convertEncoderPosition(value)))
        self.ui.PIP_scrollbar.valueChanged.connect(lambda value: self.setPosition(id = 3, position = self.convertEncoderPosition(value)))
        self.ui.MCP_scrollbar.valueChanged.connect(lambda value: self.setPosition(id = 2, position = self.convertEncoderPosition(value)))
        self.ui.ABD_scrollbar.valueChanged.connect(lambda value: self.setPosition(id = 1, position = self.convertEncoderPosition(value)))

        encoder_validator = QIntValidator(0, 4096) # Ensure text box only accepts integers
        self.ui.DIP_goalpos_textbox.setValidator(encoder_validator)
        self.ui.PIP_goalpos_textbox.setValidator(encoder_validator)
        self.ui.MCP_goalpos_textbox.setValidator(encoder_validator)
        self.ui.ABD_goalpos_textbox.setValidator(encoder_validator)

        self.ui.DIP_goalpos_textbox.returnPressed.connect(lambda : self.setPosition(id = 1, position = self.ui.DIP_goalpos_textbox.text()))
        self.ui.PIP_goalpos_textbox.returnPressed.connect(lambda : self.setPosition(id = 1, position = self.ui.DIP_goalpos_textbox.text()))
        self.ui.MCP_goalpos_textbox.returnPressed.connect(lambda : self.setPosition(id = 1, position = self.ui.DIP_goalpos_textbox.text()))
        self.ui.ABD_goalpos_textbox.returnPressed.connect(lambda : self.setPosition(id = 1, position = self.ui.DIP_goalpos_textbox.text()))

    def convertEncoderPosition(self, percent):
        min = 0
        max = 4096
        return int(percent / 100 * max + min)

    def setPosition(self, id, position):
        if not position: position = 0
        self.controlNode.setPosition(id = id, position = int(position))

def main():
    sys.exit(Main().main(sys.argv, standalone="rqt_finger_manipulation.rqt_finger_manipulation"))