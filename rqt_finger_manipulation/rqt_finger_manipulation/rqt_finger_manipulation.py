from rqt_gui_py.plugin import Plugin
from rqt_gui.main import Main
from PyQt5.QtWidgets import QMainWindow
from rqt_finger_manipulation.main_window import main_window
import sys

class RqtPlugin(Plugin):
    def __init__(self, context):
        super(RqtPlugin, self).__init__(context)

        self._widget = QMainWindow()
        self.ui = main_window()
        self.ui.setupUi(self._widget)
        context.add_widget(self._widget)

def main():
    sys.exit(Main().main(sys.argv, standalone="rqt_finger_manipulation.rqt_finger_manipulation"))