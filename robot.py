#!/usr/bin/env python3

from typing import Optional
from commands2 import Command, CommandScheduler
from wpilib import Encoder, DriverStation, Preferences
from robotcontainer import RobotContainer
from wpilib.shuffleboard import Shuffleboard
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelSpeeds
from wpimath.controller import PIDController
from photonlibpy.photonCamera import PhotonPipelineResult
from logging import basicConfig, DEBUG, info
from commands2.timedcommandrobot import TimedCommandRobot
from wpimath.geometry import Translation2d
from math import pi
from navx import AHRS

ARM_CPR = 7  # arm motor counts per revolution


class POVDirection:
    # translations for the dpad. may be removed later
    UP = (0, 45, 270)
    DOWN = (135, 180, 225)


basicConfig(level=DEBUG)


def cap(num: float, threshold: float):
    """returns a number or a maximum value."""
    return threshold * num


class Ghost(TimedCommandRobot):
    __slots__ = ("camera", "auto_active", "arm_encoder", "subsystems", "gyro")

    def init_pid(self):
        # self.forward_pid = PIDController(0.1, 0.01, 0.05)
        self.drive_pid = PIDController(0.0025, 0, 0)
        self.arm_pid = PIDController(0.005, 0, 0)
        self.stop_pid = PIDController(0.1, 0.01, 0.05)

    def info(self, message: str):
        if (not DriverStation.isDSAttached()) or DriverStation.isFMSAttached():
            # * don't run if there's no driver station/we're in competition
            return
        info(message)

    # dio- port 0 green, port 1 yellow, port 2 blue, and port 3 white

    def robotInit(self):
        self.autonomousCommand: Optional[Command] = None
        self.subsystems = RobotContainer()

        Preferences.initBoolean("auto_active", False)
        self.auto_active = Preferences.getBoolean("auto_active", False)
        self.arm_encoder = Encoder(1, 2, False, Encoder.EncodingType.k4X)
        self.arm_encoder.setDistancePerPulse(1 / 1167.75)

        self.init_pid()

        self.dash = Shuffleboard.getTab("LiveWindow")
        self.dash.add("drivetrain", self.subsystems.drivetrain)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.subsystems.drivetrain.safety_enabled(True)

    def teleopPeriodic(self): ...

    def autonomousInit(self) -> None: ...

    def autonomousPeriodic(self) -> None:
        """
        1 rotation = ~18.84954 in.
        """
        self.subsystems.shooter.shoot()
        self.subsystems.drivetrain.stop()
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
        self.subsystems.drivetrain.stop()

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
