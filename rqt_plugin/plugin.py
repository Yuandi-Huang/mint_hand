from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from .my_ui import Ui_MainWindow

class RqtPlugin(Plugin):
    def __init__(self, context):
        super(RqtPlugin, self).__init__(context)
        
        self._widget = QWidget()
        self.ui = main_window()
        self.ui.setupUi(self._widget)
        context.add_widget(self._widget)
