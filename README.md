# MAXSwerve Python Template
The python implementation of [REV Robotics MAXSwerve Modules](https://docs.revrobotics.com/ion-build-system/motion/maxswerve) as adapted from the [REV Robotics CPP Template](https://github.com/REVrobotics/MAXSwerve-Cpp-Template) for use in an environment using RobotPy. Suitable for FIRST Robotics Teams that want to utilize swerve drive and are using Python for their robot code. As Python is now officially supported by FRC as of 2024, a python adaptation is necessary.

## Requirements
### Windows 7 (64-bit) or newer
Windows is only necessary for programming the CAN IDs using the [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/gs/install)

## Installation & Setup

### Install Python & RobotPy
Follow the [FRC Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/python-setup.html) for step by step instructions to get the environment setup for python in FRC.

### Clone this repository
The commands below will download this repository and install all of the necessary packages to your environment which will then be transferred over to the RoboRio
```console
user@user:~$ git clone https://github.com/edubu/MAXSwerve-Python-Template.git
user@user~$ py -3 -m robotpy sync
```

### Setup the SparkMax Controllers using CAN
The SparkMAX require the use of CAN to communicate with the encoders. If seeking guidance wiring with CAN, checkout [this guide](https://docs.wpilib.org/en/stable/docs/hardware/hardware-basics/can-wiring-basics.html) to help through the process.

After the wiring has been completed, you will need to connected to each SparkMax using the [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/gs/install)

Follow [this guide](https://docs.revrobotics.com/ion-build-system/motion/maxswerve) to program the SparkMax controllers to their unique CAN IDs

### Modify the SwerveDrive Parameters
The swerve drive parameters are located in constants.py and will be used to communicate with the SparkMax controllers and setup the kinematics of the chassis.

#### Change the CAN ID parameters
constants.py
```python
# SPARK MAX CAN IDs
kFrontLeftDrivingCanId = 2
kRearLeftDrivingCanId = 8
kFrontRightDrivingCanId = 4
kRearRightDrivingCanId = 6

kFrontLeftTurningCanId = 3
kRearLeftTurningCanId = 1
kFrontRightTurningCanId = 5
kRearRightTurningCanId = 7
```

#### Set the dimensions of the chassis
constants.py
```python
# Chassis configuration
kTrackWidth = 0.6953 # Distance between centers of right and left wheels on robot METERS
kWheelBase = 0.6953 # Distance between centers of front and back wheels on robot METERS
```

#### Set the pinion gear used in the MaxSwerve Module
constants.py
```python
# The MaxSwerve module can be configured with one of the three pinion gears: 12T, 13T, or 14T.
# This changes the drive speed of the module ( a pinion gear with more teeth will result in a robot that drives faster)
kDrivingMotorPinionTeeth = 13
```

## FAQs
### Do I need to use the NavX Gyroscope?
No, the NavX gyroscope was simply used because we had it available. You can use any gyro by simply changing the following to connect to your available hardware
```python
 # NavX Gyroscope
self.gyro = navx.AHRS(wpilib.SerialPort.Port.kUSB)

# Example Analog Gyroscope
self.gyro = wpilib.AnalogGyro(analogChannel)
```

### What controllers are being used?
In this template, an XboxController class is utilized to control the robot. The axis are predefined to control it and utilize SlewRateLimiters as well as a Deadband to facilitate smooth driving.


## Resources
[REV Robotics MAXSwerve Module](https://www.revrobotics.com/rev-21-3005/)\
[REV Robotics MAXSwerve Docs](https://docs.revrobotics.com/ion-build-system/motion/maxswerve)\
[REV Robotics MAXSwerve CPP Template](https://github.com/REVrobotics/MAXSwerve-Cpp-Template)\
[RobotPy Docs](https://robotpy.readthedocs.io/projects/wpilib/en/latest/index.html)