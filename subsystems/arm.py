from commands2.subsystem import Subsystem
from rev import CANSparkMax
from wpilib import Encoder, Preferences
from wpilib.shuffleboard import Shuffleboard
from constants import ArmPID
from wpimath.controller import PIDController


class Arm(Subsystem):
    __slots__ = ("motor", "encoder", "setpoint", "pid")

    def __init__(self):
        super().__init__()
        self.motor = CANSparkMax(9, CANSparkMax.MotorType.kBrushed)
        self.pid = PIDController(ArmPID.K_P, ArmPID.K_I, ArmPID.K_D)
        self.pid.setTolerance(1)

        self.encoder = Encoder(1, 2, False, Encoder.EncodingType.k4X)
        self.mode = 0
        self.encoder.setDistancePerPulse(1 / 7)
        Shuffleboard.getTab("LiveWindow").add(self.encoder)
        Shuffleboard.getTab("LiveWindow").add(self.pid)

    def get_setpoint(self) -> int:
        return self.mode * 88

    def toggle(self):
        self.mode = (self.mode + 1) % 3

    def periodic(self):
        # return # skip all the other stuff
        #! positive is up, negative is down
        error = self.get_setpoint() - self.encoder.getDistance()
        # self.motor.set(self.pid.calculate(error, self.get_setpoint()))
        if abs(error) < 1:
            self.motor.set(0)
        else:
            self.motor.set((error * ArmPID.K_P) + (0.3 * error / abs(error)))
        # self.pid.setReference(self.get_setpoint(), CANSparkMax.ControlType.kPosition)
        # print(error)
        # if abs(error) < 1:  # * if it's less than 0.1 degree off...
        #     # print("stopping motor")
        #     self.motor.set(0)  # * stop the motor (we're close enough)

        # # * if the arm is too high, keep moving down
        # elif error > 0:
        #     # print("moving down")
        #     self.motor.set(1)

        # # * if the arm is too low, keep moving up
        #     self.pid.
        # elif error < 0:
        #     # print("moving up")
        #     self.motor.set(-1)
