from commands2.subsystem import Subsystem
from rev import CANSparkMax, SparkRelativeEncoder
from wpilib import MotorControllerGroup
from wpilib.drive import MecanumDrive

class EncoderGroup:
    __slots__ = ("front", "rear")

    def __init__(self, front: CANSparkMax, rear: CANSparkMax):
        self.front: SparkRelativeEncoder = front.getEncoder()
        self.rear: SparkRelativeEncoder = rear.getEncoder()

class Mecanum(Subsystem):
    def __init__(self):
        super().__init__()

        left_rear = CANSparkMax(1, CANSparkMax.MotorType.kBrushless)
        right_rear = CANSparkMax(2, CANSparkMax.MotorType.kBrushless)
        # self.left_motors = MotorControllerGroup(left_front, left_rear)

        right_front = CANSparkMax(3, CANSparkMax.MotorType.kBrushless)
        left_front = CANSparkMax(4, CANSparkMax.MotorType.kBrushless)

        self.arm = CANSparkMax(9, CANSparkMax.MotorType.kBrushed)
        # self.arm.setIdleMode(CANSparkMax.IdleMode.kCoast)
        self.left_encoders = EncoderGroup(left_front, left_rear)

        """make them all brake when you let go of the stick"""
        left_front.setIdleMode(CANSparkMax.IdleMode.kBrake)
        left_rear.setIdleMode(CANSparkMax.IdleMode.kBrake)
        right_front.setIdleMode(CANSparkMax.IdleMode.kBrake)
        right_rear.setIdleMode(CANSparkMax.IdleMode.kBrake)

        """invert some of the motors (they just have to be like that)"""
        right_front.setInverted(True)
        right_rear.setInverted(True)

        """tell it how we want to drive"""
        self.drivetrain = MecanumDrive(left_front, left_rear, right_front, right_rear)
        self.drivetrain.setExpiration(0.1)

        """let us use the motors in the other functions (the parts that start with def like def robotInit)"""
        self.right_front = right_front
        self.left_front = left_front
        self.right_rear = right_rear
        self.left_rear = left_rear

        self.left_front.setSmartCurrentLimit(40)
        self.right_front.setSmartCurrentLimit(40)
        self.left_front.setSmartCurrentLimit(40)
        self.right_front.setSmartCurrentLimit(40)