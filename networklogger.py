import wpilib
import ntcore

import navx

"""
    Logs all informations and is the standard for Network Table
    throughout the application.

    Abstracts the network table logging interface
"""

# Get the default instance of NetworkTables that was created automatically
# when the robot program starts
inst = ntcore.NetworkTableInstance.getDefault()
inst.startServer()

class NetworkLogger:
    def __init__(self) -> None:
        # Initialize the subsystems for logging
        self.pdp = wpilib.PowerDistribution()

        # Get the table within that instance that contains the data. There can
        # be as many tables as you like and exist to make it easier to organize
        # your data. In this case, it's a table called datatable.
        self.controller_table = inst.getTable("Controller")
        self.robot_table = inst.getTable("Robot")

        # Start publishing topics within that table that correspond to the X and Y values
        # for some operation in your program.
        # The topic names are actually "/datatable/x" and "/datatable/y".

        # Controller table
        self.LJoystickXPub = self.controller_table.getDoubleTopic("LJoystickX").publish()
        self.LJoystickYPub = self.controller_table.getDoubleTopic("LJoystickY").publish()
        self.RJoystickXPub = self.controller_table.getDoubleTopic("RJoystickX").publish()
        self.RJoystickYPub = self.controller_table.getDoubleTopic("RJoystickY").publish()

        # Robot Table
        self.pdpPub = self.robot_table.getDoubleTopic("Voltage").publish()
        self.anglePub = self.robot_table.getDoubleTopic("Angle").publish()

    def log_gyro(self, gyro: navx.AHRS):
        self.anglePub.set(gyro.getAngle())
        
    def log_voltage(self):
        self.pdpPub.set(self.pdp.getVoltage())
    
    def log_controller(self, controller: wpilib.XboxController):
        self.LJoystickXPub.set(controller.getLeftX())
        self.LJoystickYPub.set(-controller.getLeftY())
        self.RJoystickXPub.set(controller.getRightX())
        self.RJoystickYPub.set(-controller.getRightY())
