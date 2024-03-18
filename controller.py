from wpilib import Joystick


class POVDirection:
    # translations for the dpad. may be removed later
    UP = (0, 45, 270)
    DOWN = (135, 180, 225)


class Controller:
    """
    object based on the logitech f310 controller in directinput mode.
    """

    __slots__ = "__controller"

    def __init__(self, joystick_number: int):
        self.__controller: Joystick = Joystick(joystick_number)

    def get_left_stick(self) -> float:
        return self.__controller.getY() * -1

    def get_right_stick(self) -> float:
        return self.__controller.getZ()

    def get_button_x(self) -> bool:
        return self.__controller.getRawButton(1)

    def get_button_a(self) -> bool:
        return self.__controller.getRawButton(2)

    def get_button_b(self) -> bool:
        return self.__controller.getRawButton(3)

    def get_button_y(self) -> bool:
        return self.__controller.getRawButton(4)

    def get_left_bumper(self) -> bool:
        return self.__controller.getRawButton(5)

    def get_right_bumper(self) -> bool:
        return self.__controller.getRawButton(6)

    def get_left_trigger(self) -> bool:
        return self.__controller.getRawButton(7)

    def get_right_trigger(self) -> bool:
        return self.__controller.getRawButton(8)

    def get_dpad(self) -> int:
        return self.__controller.getPOV()
