import math

import wpilib
import wpimath.geometry
import wpimath.kinematics
import wpimath.filter
import wpimath.units
import ntcore
import navx

import swervemodule
import constants


class DriveSubsystem:
    """
    Represents a swerve drive style drivetrain.
    """
    def __init__(self) -> None:
        self.kDriveKinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d(constants.kWheelBase / 2, constants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(constants.kWheelBase / 2, -constants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(-constants.kWheelBase / 2, constants.kTrackWidth / 2),
            wpimath.geometry.Translation2d(-constants.kWheelBase / 2, -constants.kTrackWidth / 2),
        )

        self.frontLeft = swervemodule.SwerveModule(
            constants.kFrontLeftDrivingCanId, 
            constants.kFrontLeftTurningCanId,
            constants.kFrontLeftChassisAngularOffset
        )
        self.rearLeft = swervemodule.SwerveModule(
            constants.kRearLeftDrivingCanId,
            constants.kRearLeftTurningCanId,
            constants.kRearLeftChassisAngularOffset
        )
        self.frontRight = swervemodule.SwerveModule(
            constants.kFrontRightDrivingCanId,
            constants.kFrontRightTurningCanId,
            constants.kFrontRightChassisAngularOffset
        )
        self.rearRight = swervemodule.SwerveModule(
            constants.kRearRightDrivingCanId,
            constants.kRearRightTurningCanId,
            constants.kRearRightChassisAngularOffset
        )

        # the gyro sensor
        self.gyro = navx.AHRS(wpilib.SerialPort.Port.kUSB)

        # Slew rate filter variables for controlling the lateral acceleration
        self.currentRotation = 0.0
        self.currentTranslationDir = 0.0
        self.currentTranslateMag = 0.0

        self.magLimiter = wpimath.filter.SlewRateLimiter(constants.kMagnitudeSlewRate / 1)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(constants.kRotationalSlewRate / 1)

        self.prevTime = ntcore._now() * pow(1, -6) # secodns

        # Odometry class for tracking robot pose
        # 4 defines the number of modules
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kDriveKinematics,
            wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.getAngle())),
            (self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()),
            wpimath.geometry.Pose2d()
        )



    def periodic(self):
        self.odometry.update(
            wpimath.geometry.Rotation2d(wpimath.units.degreesToRadians(self.gyro.getAngle())),
            (self.frontLeft.getPosition(), self.frontRight.getPosition(), self.rearLeft.getPosition(), self.rearRight.getPosition()),
        )

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        rateLimit: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param rateLimit: Whether to enable rate limiting for smoother control
        :param periodSeconds: Time
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                )
                if fieldRelative
                else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])
    
    def setX() -> None:
        pass

    def resetEncoders() -> None:
        pass

    def setModuleStates(desiredStates: list[wpimath.kinematics.SwerveModuleState]) -> None:
        pass

    # Returns the robot's heading in degrees from 180 to 180
    def getHeading() -> float:
        pass

    # Zeroes the heading of the robot
    def zeroHeading() -> None:
        pass 

    # Returns the turn rate of the robot in degrees per second
    def getTurnRate() -> float:
        pass

    # returns the currently-estimated pose
    def getPose() -> wpimath.geometry.Pose2d:
        pass
    
    # Resets the odometry to the specified pose
    def resetOdometry(pose: wpimath.geometry.Pose2d):
        pass

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
