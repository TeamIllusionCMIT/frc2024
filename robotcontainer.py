#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
from wpilib import XboxController

from commands2.button import CommandXboxController as Controller
from commands2 import InstantCommand, Command, RunCommand, SelectCommand, PrintCommand

from subsystems.mecanum import Mecanum
from subsystems.shooter import Shooter
from subsystems.odometry import Odometry
from subsystems.vision import Vision

from wpimath.geometry import Rotation2d

from enum import Enum, auto

from navx import AHRS


class DummyGyro:
    def getRotation2d(self) -> Rotation2d:
        return Rotation2d(0, 0)

    def getAngle(self) -> float:
        return 0


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    __slots__ = (
        "drivetrain",
        "shooter",
        "gyro",
        "vision",
        "odometry",
        "controller",
        "select_command",
    )

    class CommandSelector(Enum):
        NONE = auto()
        ONE = auto()
        TWO = auto()
        THREE = auto()

    def select(self) -> CommandSelector:
        return self.CommandSelector.NONE

    def __init__(self, is_fake: bool):
        """The container for the robot. Contains subsystems, OI devices, and commands."""

        if is_fake:
            self.gyro = DummyGyro()
        else:
            self.gyro = AHRS.create_spi()  # ! since this breaks tests for some reason

        # initialize the robot's subsystems
        self.vision = Vision("Global_Camera_Shutter")
        self.drivetrain = Mecanum()
        self.shooter = Shooter()

        # * so the linter doesn't get mad at me
        self.odometry = Odometry(self.gyro, self.drivetrain, self.vision)  # type: ignore

        # The driver's controller
        self.controller = XboxController(0)

        # Configure the button bindings
        self.configureButtonBindings()

        # * sets the default command, which will run when
        # * no other command is running (for the drivetrain)
        self.drivetrain.setDefaultCommand(
            RunCommand(
                lambda: self.drivetrain.drive(
                    -self.controller.getLeftY(),
                    self.controller.getLeftX(),
                    self.controller.getRightX(),
                ),
                self.drivetrain,
            )
        )

        self.select_command = SelectCommand(
            # Maps selector values to commands
            {
                self.CommandSelector.ONE: PrintCommand("Command one was selected!"),
                self.CommandSelector.TWO: PrintCommand("Command two was selected!"),
                self.CommandSelector.THREE: PrintCommand("Command three was selected!"),
            },
            self.select,
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """
        # * left trigger for shooter intake
        Controller(0).leftTrigger(0.01).onTrue(
            RunCommand(
                lambda: self.shooter.intake(),
                self.shooter,
            )
        ).onFalse(
            InstantCommand(
                lambda: self.shooter.stop(),
                self.shooter,
            )
        )

        # * right trigger to shoot
        Controller(0).rightTrigger(0.01).onTrue(
            RunCommand(
                lambda: self.shooter.shoot(),
                self.shooter,
            )
        ).onFalse(
            InstantCommand(
                lambda: self.shooter.stop(),
                self.shooter,
            )
        )

        # * left bumper to arm intake
        Controller(0).leftBumper().onTrue(
            RunCommand(
                lambda: self.shooter.arm_intake(),
                self.shooter,
            )
        ).onFalse(
            InstantCommand(
                lambda: self.shooter.arm_stop(),
                self.shooter,
            )
        )

        # * right bumper for arm spit
        Controller(0).rightBumper().onTrue(
            RunCommand(
                lambda: self.shooter.arm_spit(),
                self.shooter,
            )
        ).onFalse(
            InstantCommand(
                lambda: self.shooter.arm_stop(),
                self.shooter,
            )
        )

        # * A button to invert the drivetrain
        Controller(0).start().onTrue(
            InstantCommand(
                lambda: self.drivetrain.invert(),
                self.drivetrain,
            )
        )

        # * control arm with dpad
        # Controller(0).povUp().or_(Controller(0).povUpLeft()).or_(Controller(0).povUpRight()).onTrue(
        #     lambda: arm_up() # (this is positive 1)
        # )

        ...

    def getAutonomousCommand(self) -> Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return self.select_command
