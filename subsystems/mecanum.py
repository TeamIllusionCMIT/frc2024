from commands2.subsystem import Subsystem
from rev import CANSparkMax, SparkRelativeEncoder
from wpilib.drive import MecanumDrive
from wpimath.units import inchesToMeters


class EncoderGroup:
    __slots__ = ("front", "rear")

    def __init__(self, front: CANSparkMax, rear: CANSparkMax):
        self.front: SparkRelativeEncoder = front.getEncoder()
        self.rear: SparkRelativeEncoder = rear.getEncoder()

    def set_conversion_factor(self, factor: float):
        self.front.setPositionConversionFactor(factor)
        self.rear.setPositionConversionFactor(factor)


class Mecanum(Subsystem):
    """drivetrain subsystem."""

    REV_CPR = 42
    WHEEL_CIRCUMFERENCE = inchesToMeters(3.5)

    __slots__ = (
        "inversion_factor",
        "arm",
        "drivetrain",
        "gyro",
        "left_encoders",
        "right_encoders",
    )

    def deadzone(self, value: float, deadzone: float = 0.075):
        """
        returns 0 if the value is within the deadzone, otherwise returns the value

        combats joystick drift
        """
        return 0 if abs(value) < deadzone else value

    def __init__(self, max_output: float = 1.0):
        super().__init__()

        # self.arm.setIdleMode(CANSparkMax.IdleMode.kCoast)

        # * start off not inveretd
        self.inversion_factor = 1

        # * initialize the motors
        left_rear = CANSparkMax(1, CANSparkMax.MotorType.kBrushless)
        right_rear = CANSparkMax(2, CANSparkMax.MotorType.kBrushless)

        right_front = CANSparkMax(3, CANSparkMax.MotorType.kBrushless)
        left_front = CANSparkMax(4, CANSparkMax.MotorType.kBrushless)

        # * when we're not driving, brake
        # ? do we want this? driving seems much smoother when coasting
        left_front.setIdleMode(CANSparkMax.IdleMode.kBrake)
        left_rear.setIdleMode(CANSparkMax.IdleMode.kBrake)

        right_front.setIdleMode(CANSparkMax.IdleMode.kBrake)
        right_rear.setIdleMode(CANSparkMax.IdleMode.kBrake)

        # * invert the right side (they just are like this)
        right_front.setInverted(True)
        right_rear.setInverted(True)

        # * basically power save mode for the motors
        left_front.setSmartCurrentLimit(40)
        right_front.setSmartCurrentLimit(40)
        left_rear.setSmartCurrentLimit(40)
        right_rear.setSmartCurrentLimit(40)

        self.left_encoders = EncoderGroup(left_front, left_rear)
        self.right_encoders = EncoderGroup(right_front, right_rear)

        # * make the encoders return meters
        self.left_encoders.set_conversion_factor(
            self.REV_CPR / self.WHEEL_CIRCUMFERENCE
        )
        self.right_encoders.set_conversion_factor(
            self.REV_CPR / self.WHEEL_CIRCUMFERENCE
        )

        """tell it how we want to drive"""
        self.drivetrain = MecanumDrive(left_front, left_rear, right_front, right_rear)
        self.drivetrain.setExpiration(0.1)

        self.drivetrain.setMaxOutput(max_output)

    def invert(self):
        """invert the drivetrain (for when aiming to shoot, since the shooter is in the back)"""
        self.inversion_factor *= -1

    def drive(self, forward_motion: float, side_motion: float, rotation: float):
        """simpler drive function (also better labeled)

        args:
            forward_motion (float): the forward/backward input
            side_motion (float): the left/right input
            rotation (float): the rotation input
        """
        self.drivetrain.driveCartesian(
            self.deadzone(forward_motion) * self.inversion_factor,
            self.deadzone(side_motion) * self.inversion_factor,
            self.deadzone(rotation) * self.inversion_factor,
        )

    def stop(self):
        """stop the drivetrain, pretty straightforward"""
        self.drivetrain.stopMotor()

    def max_output(self, value: float):
        """set the maximum possible output of the drivetrain

        args:
            value (float): the maximum output. between -1 and 1.
        """
        self.drivetrain.setMaxOutput(value)

    def safety_enabled(self, value: bool):
        """set the safety of the drivetrain

        args:
            value (bool): whether or not the safety should be enabled"""
        self.drivetrain.setSafetyEnabled(value)
