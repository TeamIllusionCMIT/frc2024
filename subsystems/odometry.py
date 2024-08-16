from typing import Any
from commands2.subsystem import Subsystem
from wpimath.kinematics import (
    MecanumDriveKinematics,
    MecanumDriveOdometry,
    MecanumDriveWheelSpeeds,
    MecanumDriveWheelPositions,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import Field2d, Timer, DriverStation
from wpilib.shuffleboard import Shuffleboard

from navx import AHRS
from subsystems.vision import Vision
from subsystems.mecanum import Mecanum

from constants import LINEAR_SPEED
from logging import basicConfig, DEBUG, info as linfo

basicConfig(level=DEBUG)


def info(message: Any):
    if (not DriverStation.isDSAttached()) or DriverStation.isFMSAttached():
        # * don't run if there's no driver station/we're in competition
        return
    linfo(message)


class Odometry(Subsystem):
    """basically robot gps. the robot thinks about how much the wheels have turned and its original position to figure out where it is now."""

    def __init__(self, gyro: AHRS, drivetrain: Mecanum, vision: Vision):
        self.vision_timer = Timer()  # cooldown for vision updates
        self.drivetrain = drivetrain

        self.kinematics = MecanumDriveKinematics(
            frontLeftWheel=Translation2d().fromFeet(-1.7391667, 2 / 3),
            frontRightWheel=Translation2d().fromFeet(1.7391667, 2 / 3),
            rearLeftWheel=Translation2d().fromFeet(-1.7391667, -2 / 3),
            rearRightWheel=Translation2d().fromFeet(1.7391667, -2 / 3),
        )

        self.wheel_positions = MecanumDriveWheelPositions()

        # Example differential drive wheel speeds: 2 meters per second
        # for the left side, 3 meters per second for the right side.
        self.wheel_speeds = MecanumDriveWheelSpeeds(
            LINEAR_SPEED, LINEAR_SPEED, LINEAR_SPEED, LINEAR_SPEED
        )

        # Convert to chassis speeds.
        self.chassis_speeds = self.kinematics.toChassisSpeeds(self.wheel_speeds)
        self.gyro = gyro

        self.odometry = MecanumDriveOdometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            self.wheel_positions,
            robot := Pose2d(5.0, 13.5, Rotation2d()),
        )
        self.field = Field2d()
        self.field.setRobotPose(robot)
        dash = Shuffleboard.getTab("LiveWindow")
        dash.add("field", self.field)

        self.update()
        self.vision_timer.start()
        self.vision = vision

    def update(self):
        """updates the odometry and field pose."""
        self.odometry.update(self.gyro.getRotation2d(), self.update_wheel_positions())
        self.field.setRobotPose(self.odometry.getPose())

    def periodic(self):
        """updates the odometry periodically, and uses vision to update the field pose once a second."""
        if not self.vision or not self.vision_timer.advanceIfElapsed(1):
            self.update()
            return
        if self.vision_timer.advanceIfElapsed(1):
            info(self.gyro.getRotation2d().degrees())
            estimated_pose = self.vision.estimate_pose()
            if estimated_pose:  # * if we have a pose estimation from photonvision...
                pose = estimated_pose.toPose2d()
                self.odometry.resetPosition(  # * reset the odometry to the new pose
                    pose.rotation(),  # * current true rotation
                    self.update_wheel_positions(),  # * current wheel positions
                    pose,  # * current true position
                )
                self.field.setRobotPose(pose)
                self.vision.driver_mode = True  # * set the driver mode to true again
                return  # * don't update the odometry if we have a better estimate

    def get_pose(self) -> Pose2d:
        """get the current known robot pose

        returns:
            Pose2d: the current robot pose
        """
        return self.odometry.getPose()

    def update_wheel_positions(self) -> MecanumDriveWheelPositions:
        """update the robot's wheel positions.

        returns:
            MecanumDriveWheelPositions: the updated wheel positions object
        """
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
        return self.wheel_positions
