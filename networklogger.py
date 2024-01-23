import wpilib
from networktables import NetworkTables, NetworkTable

"""
    Logs all informations and is the standard for Network Table
    throughout the application.

    Abstracts the network table logging interface
"""
class NetworkLogger:
    def __init__(self, controller: wpilib.XboxController) -> None:
        # Initialize the subsystems for logging
        self.controller = controller

        # Initialize the network table with predefined tables
        NetworkTables.initialize()
        self.sd: NetworkTable = NetworkTables.getTable("SmartDashboard")
    
    def log_controller(self):
        self.sd.putNumber("LJoystickX", self.controller.getLeftX())
        self.sd.putNumber("LJoystickY", self.controller.getLeftY())
        self.sd.putNumber("RJoystickX", self.controller.getRightX())
        self.sd.putNumber("RJoystickY", self.controller.getRightY())