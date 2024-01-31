import wpilib

import constants
import intakesubsystem
import shootersubsytem
import liftsubsystem

class ControlSubsystem:
    """
        Represents the entire subsystem for the intake and shooter mechanism
        This coupling is necessary since shooter may require cooperation with the intake
    """
    def __init__(self) -> None:
        self.shooter = shootersubsytem.ShooterSubsystem()
        self.intake = intakesubsystem.IntakeSubsystem()
        self.lift = liftsubsystem.LiftSubsystem()

