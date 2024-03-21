from commands2 import CommandScheduler
from wpilib import DriverStation, Preferences
from robotcontainer import RobotContainer
from logging import basicConfig, DEBUG, info
from commands2.timedcommandrobot import TimedCommandRobot

basicConfig(level=DEBUG)


class Ghost(TimedCommandRobot):
    __slots__ = ("auto_active", "subsystems")

    def info(self, message: str):
        if (not DriverStation.isDSAttached()) or DriverStation.isFMSAttached():
            # * don't run if there's no driver station/we're in competition
            return
        info(message)

    # dio- port 0 green, port 1 yellow, port 2 blue, and port 3 white

    def robotInit(self):
        self.subsystems = RobotContainer(self.isSimulation())

        # * allows the robot to remember + the dashboard to configure whether or not autonomous should run
        Preferences.initBoolean("auto_active", False)
        self.auto_active = Preferences.getBoolean("auto_active", False)

    def teleopInit(self):
        """executed when teleoperated mode is enabled. this function turns on the controller's safety features to stop us from breaking things."""
        self.subsystems.drivetrain.safety_enabled(True)

    def teleopPeriodic(self): ...

    def teleopExit(self) -> None:
        self.subsystems.arm.mode = 0

    def autonomousInit(self) -> None: ...

    def autonomousPeriodic(self) -> None:
        if self.auto_active:
            self.subsystems.shooter.shoot()
            self.subsystems.drivetrain.stop()

    def autonomousExit(self) -> None:
        self.subsystems.drivetrain.stop()
        self.subsystems.arm.mode = 0

    def testInit(self) -> None:
        CommandScheduler.getInstance().cancelAll()
