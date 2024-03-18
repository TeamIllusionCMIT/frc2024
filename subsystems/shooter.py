from commands2.subsystem import Subsystem
from rev import CANSparkMax, SparkRelativeEncoder
from wpilib import MotorControllerGroup
from wpilib.drive import MecanumDrive


class Shooter(Subsystem):
    def __init__(self):
        super().__init__()

        intake_bot_left = CANSparkMax(5, CANSparkMax.MotorType.kBrushless)
        intake_top_left = CANSparkMax(6, CANSparkMax.MotorType.kBrushless)
        intake_bot_right = CANSparkMax(7, CANSparkMax.MotorType.kBrushless)
        intake_top_right = CANSparkMax(8, CANSparkMax.MotorType.kBrushless)

        intake_top_left.setInverted(True)
        intake_bot_right.setInverted(True)

        shooter_left = CANSparkMax(10, CANSparkMax.MotorType.kBrushed)
        shooter_right = CANSparkMax(11, CANSparkMax.MotorType.kBrushless)
        shooter_right.setInverted(False)

        shooter_left.setIdleMode(CANSparkMax.IdleMode.kBrake)
        shooter_right.setIdleMode(CANSparkMax.IdleMode.kBrake)

        top_group = MotorControllerGroup(intake_top_left, intake_top_right)
        bottom_group = MotorControllerGroup(intake_bot_left, intake_bot_right)

        shooter_group = MotorControllerGroup(shooter_left, shooter_right)

        self.intake_top_right = intake_top_right
        self.intake_top_left = intake_top_left
        self.intake_bot_right = intake_bot_right
        self.intake_bot_left = intake_bot_left

        self.shooter_left = shooter_left
        self.shooter_right = shooter_right

        """let us use the motor control groups in the other functions (the parts that start with def like def robotInit)"""
        self.top_group = top_group
        self.bottom_group = bottom_group
        self.shooter_group = shooter_group

    def ingest(self):
        # print("Swallowing the whole note.")
        self.bottom_group.set(0.25)
        self.top_group.set(-0.25)

    def throw(self):
        # print("Regurgitating the whole note.")
        self.bottom_group.set(-0.5)
        self.top_group.set(0.5)
