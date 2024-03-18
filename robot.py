#!/usr/bin/env python3

from wpilib import (
    Encoder,
    TimedRobot,
    MotorControllerGroup,
    run,
    Timer,
    XboxController,
    DriverStation,
)
from wpilib.shuffleboard import Shuffleboard
from rev import CANSparkMax, SparkRelativeEncoder
from wpilib.drive import MecanumDrive
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelSpeeds
from wpimath.filter import SlewRateLimiter
from math import pi
from wpimath.controller import PIDController
from photonlibpy.photonCamera import PhotonCamera, PhotonPipelineResult
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from logging import basicConfig, DEBUG, info
from photonutils import PhotonUtils
from wpimath.geometry import Translation2d, Transform3d, Pose3d
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField

basicConfig(level=DEBUG)

MOTOR_FREE_SPEED = 5676
LINEAR_SPEED = ((2 * pi) * 7.5 * (MOTOR_FREE_SPEED / 60)) / 100  # in m/s
ARM_CPR = 7  # arm motor counts per revolution


class POVDirection:
    # translations for the dpad. may be removed later
    UP = (0, 45, 270)
    DOWN = (135, 180, 225)


def cap(num: float, threshold: float):
    """returns a number or a maximum value."""
    return threshold * num

class Ghost(TimedRobot):
    __slots__ = (
        "factor",
        "auto_active",
        "left_front",
        "left_rear",
        "right_front",
        "right_rear",
        "stop_pid",
        "drivetrain",
        "controller",
        "right_encoders",
        "left_encoders",
        "drive_pid",
        "arm_pid",
        "camera",
        "limiter",
        "pose_estimator",
        "dash",
        "arm_encoder",
        "intake_top_left",
        "intake_top_right",
        "intake_bot_left",
        "intake_bot_right",
        "arm",
        "shooter_left",
        "shooter_right",
        "top_group",
        "bottom_group",
        "shooter_group",
        "__arm_state",
        "arm_cycling",
        "arm_timer",
    )

    def init_pid(self):
        # self.forward_pid = PIDController(0.1, 0.01, 0.05)
        self.drive_pid = PIDController(0.0025, 0, 0)
        self.arm_pid = PIDController(0.005, 0, 0)
        self.stop_pid = PIDController(0.1, 0.01, 0.05)

    def init_vision(self):
        # CameraServer()
        self.camera = PhotonCamera("Global_Camera_Shutter")
        self.pose_estimator = PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagField.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            self.camera,
            Transform3d(Pose3d(), Pose3d()),
        )

    def init_kinematics(self):
        # unfinished
        kinematics = MecanumDriveKinematics(
            frontLeftWheel=Translation2d().fromFeet(-1.7391667, 2 / 3),
            frontRightWheel=Translation2d().fromFeet(1.7391667, 2 / 3),
            rearLeftWheel=Translation2d().fromFeet(-1.7391667, -2 / 3),
            rearRightWheel=Translation2d().fromFeet(1.7391667, -2 / 3),
        )

        # Example differential drive wheel speeds: 2 meters per second
        # for the left side, 3 meters per second for the right side.
        wheel_speeds = MecanumDriveWheelSpeeds(
            LINEAR_SPEED, LINEAR_SPEED, LINEAR_SPEED, LINEAR_SPEED
        )

        # Convert to chassis speeds.
        self.chassis_speeds = kinematics.toChassisSpeeds(wheel_speeds)

    def info(self, message: str):
        if (not DriverStation.isDSAttached()) or DriverStation.isFMSAttached():
            return
        info(message)

    # dio- port 0 green, port 1 yellow, port 2 blue, and port 3 white

    def robotInit(self):
        self.factor = 1
        self.auto_active = True
        self.arm_encoder = Encoder(1, 2, False, Encoder.EncodingType.k4X)
        self.arm_encoder.setDistancePerPulse(1 / 1167.75)

        self.init_motors()
        self.init_vision()
        self.init_pid()

        self.controller = XboxController(0)
        self.dash = Shuffleboard.getTab("LiveWindow")
        self.dash.add("drivetrain", self.drivetrain)
        self.dash.add("drive pid", self.drive_pid)
        self.dash.add("arm pid", self.arm_pid)
        self.dash.add("auto active?", self.auto_active)
        self.__arm_state = 2

        self.arm_timer = Timer()
        self.drivetrain.setMaxOutput(0.75)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        # areas = self.areasSub.get()
        # print("Areas:", areas)
        # self.dash.add(self.forward_pid)

        """Runs the motors with mecanum steering"""
        try:
            if self.controller.getStartButtonPressed():
                self.factor = -self.factor
        except Exception:
            pass
        try:
            if self.controller.getLeftTriggerAxis() > 0:
                self.shooter_group.set(-0.5)
                # return # remove this to instead aim
                # side_motion = 0
                # forward_motion = 0
                # rotate_motion = 0
                # result: PhotonPipelineResult = self.camera.getLatestResult()
                # if targets := result.getTargets():
                #     distance = PhotonUtils.calculateDisanceToTargetMeters(
                #         0.73, 1.53, 3.14, targets[0].getPitch() * (pi / 180)
                #     )
                #     if "distance from target" not in [
                #         x.getTitle() for x in self.dash.getComponents()
                #     ]:
                #         self.dash.add(distance)

                #     side_motion = cap(
                #         -self.drive_pid.calculate(targets[0].getYaw(), 0), 0.75
                #     )
                #     """note to self: find the ideal area of the apriltag"""
                #     forward_motion = cap(
                #         -self.drive_pid.calculate(distance, 0.88335), 0.75
                #     )

                # self.drivetrain.driveCartesian(
                #     forward_motion, side_motion, rotate_motion
                # )
            else:
                self.drivetrain.setMaxOutput(1)
                self.drivetrain.driveCartesian(
                    # self.limiter.calculate(-self.controller.getLeftY()),
                    -self.controller.getLeftY() * self.factor if abs(self.controller.getLeftY()) > 0.075 else 0,
                    # self.limiter.calculate(self.controller.getLeftX()),
                    self.controller.getLeftX() * self.factor if abs(self.controller.getLeftX()) > 0.075 else 0,
                    (self.controller.getRightX()),
                )
        except Exception:
            self.info("chat is it cooked")
            try:
                self.drivetrain.stopMotor()
            except Exception:
                pass
        try:
            if self.controller.getLeftBumper():
                self.ingest()
            elif self.controller.getRightBumper():
                self.throw()
            else:
                self.bottom_group.stopMotor()
                self.top_group.stopMotor()
        except Exception:
            ...
        try:
            if self.controller.getPOV() in POVDirection.UP:
                if self.arm_timer.get() == 0:
                    # self.info("starting timer")
                    self.arm_timer.start()
                    self.arm.set(1)
                elif self.arm_timer.hasElapsed(1):
                    # self.info("time passed")
                    self.arm.set(0)
                    self.arm_timer.reset()
                else:
                    self.arm.set(1)
                return
                target_position = 0
                # target_position = self.__arm_state * 90

                measurement = self.arm_encoder.getDistance()
                error = target_position - measurement  # calculate error

                if (error) < 0.1:
                    self.arm.set(0)
                    self.__arm_state = (self.__arm_state + 1) % 2
                else:
                    self.arm.set(-1)
            elif self.controller.getPOV() in POVDirection.DOWN:
                self.arm.set(-1)
                return
                target_position = 1
                # target_position = self.__arm_state * 90

                measurement = self.arm_encoder.getDistance()
                error = target_position - measurement
                self.info(f"{target_position} - {measurement} = {(error)}")
                if (error) < 0.1:
                    self.arm.set(0)
                    self.__arm_state = (self.__arm_state + 1) % 2
                    self.arm_encoder.reset()
                else:
                    self.arm.set(1)
            else:
                self.arm.set(0)
        except Exception:
            pass
        try:
            if self.controller.getRightTriggerAxis() > 0:
                # print("Right Bumper Pressed.")
                self.shooter_group.set(0.5)
            else:
                self.shooter_group.stopMotor()
        except Exception:
            pass
        try:
            if self.controller.getBButton():
                self.shooter_group.set()
        except Exception:
            pass

    def autonomousInit(self) -> None:
        # self.drivetrain.stopMotor()
        # self.state.auto_begin_time = time_ns()

        # self.dash.addBoolean("see tag?", bool(targets))
        # self.dash.addFloat("pid forward output", forward_motion)
        # self.dash.addFloat("pid side output", side_motion)

        ...

    def autonomousPeriodic(self) -> None:
        """
        1 rotation = ~18.84954 in.
        """
        self.throw()
        self.drivetrain.stopMotor()
        # self.shooter_group.setVoltage(12)
        return
        result: PhotonPipelineResult = self.camera.getLatestResult()
        forward_motion: float = 0
        side_motion: float = 0

        if targets := result.getTargets():
            # pitch is down/up
            # yaw is left/right
            # side_pid is a pid controller. basically, it will try to get the yaw (distance from the center of the frame) to 0.
            # we're capping it so that the maximum will be 1
            side_motion = cap(-self.drive_pid.calculate(targets[0].getYaw(), 0), 0.3)
            # forward_motion = self.forward_pid.calculate(targets[0].getArea(), 1.6)
            # distance = PhotonUtils.estimateCameraToTargetTranslation()
            ...
        # lebron = "fortnite"
        # print(lebron)
        # if self.getPeriod() > 1:
        #     self.info(f"forward: {forward_motion}")
        #     self.info(f"side: {side_motion}")

        self.drivetrain.driveCartesian(forward_motion, side_motion, 0)
        # self.dash.add("drivetrain", self.drivetrain)
        # if self.state.auto_complete:
        #     self.drivetrain.arcadeDrive(0,0)
        #     return
        # run_speed = 0.75
        # self.drivetrain.arcadeDrive(run_speed, 0)
        # if (time_ns() - self.state.auto_begin_time)*1e-9 >= 1: self.state.auto_complete = True
        ...

    def autonomousExit(self) -> None:
        self.drivetrain.stopMotor()


if __name__ == "__main__":
    run(Ghost)
