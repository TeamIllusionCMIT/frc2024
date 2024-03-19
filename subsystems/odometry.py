from commands2.subsystem import Subsystem
from wpimath.kinematics import (
    MecanumDriveKinematics,
    MecanumDriveOdometry,
    MecanumDriveWheelSpeeds,
    MecanumDriveWheelPositions,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import Field2d
from wpilib.shuffleboard import Shuffleboard

from math import pi
from navx import AHRS
from subsystems.vision import Vision
from subsystems.mecanum import Mecanum


class Odometry(Subsystem):

    MOTOR_FREE_SPEED = 5676
    LINEAR_SPEED = ((2 * pi) * 7.5 * (MOTOR_FREE_SPEED / 60)) / 100  # in m/s

    def __init__(self, gyro: AHRS, drivetrain: Mecanum, vision: Vision):
        self.drivetrain = drivetrain

        self.kinematics = MecanumDriveKinematics(
            frontLeftWheel=Translation2d().fromFeet(-1.7391667, 2 / 3),
            frontRightWheel=Translation2d().fromFeet(1.7391667, 2 / 3),
            rearLeftWheel=Translation2d().fromFeet(-1.7391667, -2 / 3),
            rearRightWheel=Translation2d().fromFeet(1.7391667, -2 / 3),
        )

        self.wheel_positions = MecanumDriveWheelPositions()
        self.wheel_positions.frontLeft = drivetrain.left_encoders.front.getPosition()
        self.wheel_positions.rearRight = drivetrain.right_encoders.rear.getPosition()
        self.wheel_positions.frontRight = drivetrain.right_encoders.front.getPosition()
        self.wheel_positions.rearLeft = drivetrain.left_encoders.rear.getPosition()

        # Example differential drive wheel speeds: 2 meters per second
        # for the left side, 3 meters per second for the right side.
        self.wheel_speeds = MecanumDriveWheelSpeeds(
            self.LINEAR_SPEED, self.LINEAR_SPEED, self.LINEAR_SPEED, self.LINEAR_SPEED
        )

        # Convert to chassis speeds.
        self.chassis_speeds = self.kinematics.toChassisSpeeds(self.wheel_speeds)
        self.gyro = gyro

        self.odometry = MecanumDriveOdometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.wheel_positions,
            Pose2d(5.0, 13.5, Rotation2d()),
        )
        self.field = Field2d()
        dash = Shuffleboard.getTab("LiveWindow")
        # dash.add("odometry", self.odometry)
        dash.add("field", self.field)
        dash.add("gyro", gyro.getAngle())

        self.update()

    def update(self):
        self.wheel_positions.frontLeft = (
            self.drivetrain.left_encoders.front.getPosition()
        )
        self.wheel_positions.rearRight = (
            self.drivetrain.right_encoders.rear.getPosition()
        )
        self.wheel_positions.frontRight = (
            self.drivetrain.right_encoders.front.getPosition()
        )
        self.wheel_positions.rearLeft = self.drivetrain.left_encoders.rear.getPosition()
        self.odometry.update(self.gyro.getRotation2d(), self.wheel_positions)
        self.field.setRobotPose(self.odometry.getPose())

    def periodic(self):
        self.update()

    def get_pose(self) -> Pose2d:
        return self.odometry.getPose()
