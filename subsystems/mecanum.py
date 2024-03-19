from commands2.subsystem import Subsystem
from rev import CANSparkMax, SparkRelativeEncoder
from wpilib.drive import MecanumDrive
from wpimath.units import inchesToMeters


class EncoderGroup:
    __slots__ = ("front", "rear")

    def __init__(self, front: CANSparkMax, rear: CANSparkMax):
        self.front: SparkRelativeEncoder = front.getEncoder()
        self.rear: SparkRelativeEncoder = rear.getEncoder()


class Mecanum(Subsystem):
    REV_CPR = 42
    WHEEL_CIRCUMFERENCE = inchesToMeters(3.5)


    __slots__ = ("inversion_factor", "arm", "drivetrain" "gyro", "left_front", "right_front", "left_rear", "right_rear")

    def deadzone(self, value: float, deadzone: float = 0.075):
        """
        returns 0 if the value is within the deadzone, otherwise returns the value

        combats joystick drift
        """
        return 0 if abs(value) < deadzone else value

    def __init__(self, max_output: float = 1.0):
        super().__init__()

        self.arm = CANSparkMax(9, CANSparkMax.MotorType.kBrushed)
        # self.arm.setIdleMode(CANSparkMax.IdleMode.kCoast)

        self.inversion_factor = 1  # start off not inveretd

        self.left_rear = CANSparkMax(1, CANSparkMax.MotorType.kBrushless)
        self.right_rear = CANSparkMax(2, CANSparkMax.MotorType.kBrushless)

        self.right_front = CANSparkMax(3, CANSparkMax.MotorType.kBrushless)
        self.left_front = CANSparkMax(4, CANSparkMax.MotorType.kBrushless)

        """make them all brake when you let go of the stick"""
        self.left_front.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.left_rear.setIdleMode(CANSparkMax.IdleMode.kBrake)

        self.right_front.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.right_rear.setIdleMode(CANSparkMax.IdleMode.kBrake)

        """invert some of the motors (they just have to be like that)"""
        self.right_front.setInverted(True)
        self.right_rear.setInverted(True)

        self.left_front.setSmartCurrentLimit(40)
        self.right_front.setSmartCurrentLimit(40)
        self.left_rear.setSmartCurrentLimit(40)
        self.right_rear.setSmartCurrentLimit(40)

        self.left_front.getEncoder().setPositionConversionFactor(self.REV_CPR / self.WHEEL_CIRCUMFERENCE)
        self.right_front.getEncoder().setPositionConversionFactor(self.REV_CPR / self.WHEEL_CIRCUMFERENCE)
        self.left_rear.getEncoder().setPositionConversionFactor(self.REV_CPR / self.WHEEL_CIRCUMFERENCE)
        self.right_rear.getEncoder().setPositionConversionFactor(self.REV_CPR / self.WHEEL_CIRCUMFERENCE)
        

        """tell it how we want to drive"""
        self.drivetrain = MecanumDrive(self.left_front, self.left_rear, self.right_front, self.right_rear)
        self.drivetrain.setExpiration(0.1)

        self.drivetrain.setMaxOutput(max_output)

    def invert(self):
        """invert the drivetrain (for when aiming to shoot, since the shooter is in the back)"""
        self.inversion_factor *= -1

    def drive(self, forward_motion: float, side_motion: float, rotation: float):
        self.drivetrain.driveCartesian(
            self.deadzone(forward_motion) * self.inversion_factor,
            self.deadzone(side_motion) * self.inversion_factor,
            self.deadzone(rotation) * self.inversion_factor,
        )

    def stop(self):
        self.drivetrain.stopMotor()

    def max_output(self, value: float):
        self.drivetrain.setMaxOutput(value)

    def safety_enabled(self, value: bool):
        self.drivetrain.setSafetyEnabled(value)
