from commands2.subsystem import Subsystem
from rev import CANSparkMax
from wpilib import Encoder
from wpilib.shuffleboard import Shuffleboard


class Arm(Subsystem):
    __slots__ = ("motor", "encoder", "setpoint")

    def __init__(self):
        super().__init__()
        self.motor = CANSparkMax(9, CANSparkMax.MotorType.kBrushed)
        self.encoder = Encoder(1, 2, False, Encoder.EncodingType.k4X)
        self.mode = 0
        self.encoder.setDistancePerPulse(1 / 7)
        Shuffleboard.getTab("LiveWindow").add(self.encoder)
        Shuffleboard.getTab("LiveWindow").add("arm setpoint", self.mode * 180)

    def get_setpoint(self) -> int:
        return self.mode * 165

    def toggle(self):
        self.mode = (self.mode + 1) % 2

    def periodic(self):
        return  # skip all the other stuff
        #! positive is up, negative is down
        error = self.get_setpoint() - self.encoder.getDistance()
        if abs(error) < 0.1:  # * if it's less than 0.1 degree off...
            self.motor.set(0)  # * stop the motor (we're close enough)

        # * if the arm is too high, keep moving down
        elif error > 0:
            self.motor.set(-0.5)

        # * if the arm is too low, keep moving up
        elif error < 0:
            self.motor.set(0.5)
