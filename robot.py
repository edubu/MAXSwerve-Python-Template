import wpilib
import wpimath
import wpilib.drive
import wpimath.filter
import wpimath.controller
import navx
import drivesubsystem
import networklogger

import constants
import controlsubsystem

# To see messages from networktables, you must setup logging
import logging

logging.basicConfig(level=logging.DEBUG)

class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        """Robot initialization function"""
        self.driverController = wpilib.XboxController(constants.kDriverControllerPort)
        self.shooterController = wpilib.XboxController(constants.kShooterControllerPort)
        self.controlSystem = controlsubsystem.ControlSubsystem()
        self.swerve = drivesubsystem.DriveSubsystem()
        self.networklogger = networklogger.NetworkLogger()

        # Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        self.logTimer = wpilib.Timer()
        self.logging_rate = 20 # Hz
    
    def autonomousInit(self) -> None:
        pass

    def autonomousPeriodic(self) -> None:
        # self.driveWithJoystick(False)
        # self.swerve.updateOdometry()
        pass

    def teleopInit(self) -> None:
        # start logging timer
        self.logTimer.start()
    

    def teleopPeriodic(self) -> None:
        # Teleop periodic logic
        self.driveWithJoystick(True)
        
        # Logic to limit PERIODIC EVENTS
        # if self.logTimer.advanceIfElapsed(1.0/self.logging_rate):
        self.swerve.periodic()
        self.networklogger.log_controller(self.controller)

        # # restart timer
        # self.logTimer.reset()

        
    
    def testPeriodic(self) -> None:
        pass

    def driveWithJoystick(self, fieldRelative: bool) -> None:
        # Get the x speed. We are inverting this because Xbox controllers return
        # negative values when we push forward.
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.driverController.getLeftY(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )

        # Get the y speed or sideways/strafe speed. We are inverting this because
        # we want a positive value when we pull to the left. Xbox controllers
        # return positive values when you pull to the right by default.
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.driverController.getLeftX(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )

        # Get the rate of angular rotation. We are inverting this because we want a
        # positive value when we pull to the left (remember, CCW is positive in
        # mathematics). Xbox controllers return positive values when you pull to
        # the right by default.
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.driverController.getRightX(), 0.08)
            )
            # * drivesubsystem.kMaxSpeed
        )


        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, rateLimit=True)

if __name__ == "__main__":
    wpilib.run(MyRobot)