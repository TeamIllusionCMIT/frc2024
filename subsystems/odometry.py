from commands2.subsystem import Subsystem
from wpimath.kinematics import (
    MecanumDriveKinematics,
    MecanumDriveOdometry,
    MecanumDriveWheelSpeeds,
    MecanumDriveWheelPositions,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.units import feetToMeters, inchesToMeters
from math import pi
from navx import AHRS
from subsystems.vision import Vision
from subsystems.mecanum import Mecanum


class Odometry(Subsystem):

    MOTOR_FREE_SPEED = 5676
    LINEAR_SPEED = ((2 * pi) * 7.5 * (MOTOR_FREE_SPEED / 60)) / 100  # in m/s

    def __init__(self, gyro: AHRS, drivetrain: Mecanum, vision: Vision):
        self.kinematics = MecanumDriveKinematics(
            frontLeftWheel=Translation2d().fromFeet(-1.7391667, 2 / 3),
            frontRightWheel=Translation2d().fromFeet(1.7391667, 2 / 3),
            rearLeftWheel=Translation2d().fromFeet(-1.7391667, -2 / 3),
            rearRightWheel=Translation2d().fromFeet(1.7391667, -2 / 3),
        )

        self.wheel_positions = MecanumDriveWheelPositions()
        self.wheel_positions.frontLeft(drivetrain.left_front.getEncoder().getPosition())
        self.wheel_positions.rearRight(
            drivetrain.right_front.getEncoder().getPosition()
        )
        self.wheel_positions.frontRight(
            drivetrain.Right_front.getEncoder().getPosition()
        )
        self.wheel_positions.rearLeft(drivetrain.Left_front.getEncoder().getPosition())

        # Example differential drive wheel speeds: 2 meters per second
        # for the left side, 3 meters per second for the right side.
        self.wheel_speeds = MecanumDriveWheelSpeeds(
            self.LINEAR_SPEED, self.LINEAR_SPEED, self.LINEAR_SPEED, self.LINEAR_SPEED
        )

        # Convert to chassis speeds.
        self.chassis_speeds = self.kinematics.toChassisSpeeds(self.wheel_speeds)

        self.odometry = MecanumDriveOdometry(
            self.kinematics,
            gyro.getRotation2d(),
            self.wheel_positions,
            Pose2d(5.0, 13.5, Rotation2d()),
        )
