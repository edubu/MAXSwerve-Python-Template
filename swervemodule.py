import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory
import rev

import constants

kWheelRadius = 0.0508
kEncoderResolution = 4096
#rev neo is 42 for encoder resolutionz: try this out
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        chassisAngularOffset: float
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        """

        """ Initialize Spark Max motor controllers"""
        self.drivingSparkMax: rev.CANSparkMax = rev.CANSparkMax(driveMotorChannel)
        self.turningSparkMax: rev.CANSparkMax = rev.CANSparkMax(turningMotorChannel)

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        self.drivingSparkMax.restoreFactoryDefaults()
        self.turningSparkMax.restoreFactoryDefaults()

        """ Initialize Spark Max encoders"""
        # Get Encoder Objects from Spark Max
        self.drivingEncoder: rev.SparkRelativeEncoder = self.drivingSparkMax.getEncoder(rev.SparkMaxRelativeEncoder.Type.kHallSensor)
        self.turningEncoder: rev.SparkAbsoluteEncoder = self.turningSparkMax.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)

        # Apply position and velocity conversion factors for the driving encoder.
        # We want these in radians and radians per second to use with WPILibs swerve APIs
        self.drivingEncoder.setPositionConversionFactor(constants.kDrivingEncoderPositionFactor)
        self.drivingEncoder.setVelocityConversionFactor(constants.kDrivingEncoderVelocityFactor)

        # Apply position and velocity conversion factors for the driving encoder.
        # We want these in radians and radians per second to use with WPILibs swerve APIs
        self.turningEncoder.setPositionConversionFactor(constants.kTurningEncoderPositionFactor)
        self.turningEncoder.setVelocityConversionFactor(constants.kTurningEncoderVelocityFactor)

        # Invert the turning encoder, since the output shaft rotates in the opposite
        # direction of the steering motor in the MAXSwerve Module.
        self.turningEncoder.setInverted(constants.kTurningEncoderInverted)

        """ Initialize PID Controllers"""
        # create spark max pid controllers
        self.drivingPIDController: rev.SparkPIDController = self.drivingSparkMax.getPIDController()
        self.turningPIDController: rev.SparkPIDController = self.turningSparkMax.getPIDController()

        # Enable PID wrap around for the turning motor. This will allow the PID
        # controller to go through 0 to get to the setpoint i.e. going from 350
        # degrees to 10 degrees will go through 0 rather than the other direction
        #  which is a longer route.
        self.turningPIDController.setPositionPIDWrappingEnabled(True)
        self.turningPIDController.setPositionPIDWrappingMinInput(
            constants.kTurningEncoderPositionPIDMinInput)
        self.turningPIDController.setPositionPIDWrappingMaxInput(
            constants.kTurningEncoderPositionPIDMaxInput)
        
        # Set the PID Controller to use the duty cycle encoder on the swerve
        # module instead of the built in NEO550 encoder.
        self.turningPIDController.setFeedbackDevice(self.turningEncoder)

        # Set the PID gains for the driving motor. Note these are example gains, and
        # you may need to tune them for your own robot!
        self.drivingPIDController.setP(constants.kDrivingP)
        self.drivingPIDController.setI(constants.kDrivingI)
        self.drivingPIDController.setD(constants.kDrivingD)
        self.drivingPIDController.setFF(constants.kDrivingFF)
        self.drivingPIDController.setOutputRange(constants.kDrivingMinOutput, constants.kDrivingMaxOutput)

        # Set the PID gains for the turning motor. Note these are example gains, and
        # you may need to tune them for your own robot!
        self.turningPIDController.setP(constants.kTurningP)
        self.turningPIDController.setI(constants.kTurningI)
        self.turningPIDController.setD(constants.kTurningD)
        self.turningPIDController.setFF(constants.kTurningFF)
        self.turningPIDController.setOutputRange(constants.kTurningMinOutput, constants.kTurningMaxOutput)

        """ Spark Max Mode Parameters"""
        self.drivingSparkMax.setIdleMode(constants.kDrivingMotorIdleMode)
        self.turningSparkMax.setIdleMode(constants.kTurningMotorIdleMode)
        self.drivingSparkMax.setSmartCurrentLimit(constants.kDrivingMotorCurrentLimit)
        self.turningSparkMax.setSmartCurrentLimit(constants.kDrivingMotorCurrentLimit)

        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations
        self.drivingSparkMax.burnFlash()
        self.turningSparkMax.burnFlash()

        # Swerve drive parameters
        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState.angle = wpimath.kinematics.SwerveModuleState(0.0, wpimath.geometry.Rotation2d(self.turningEncoder.getPosition()))
        self.drivingEncoder.setPosition(0)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)