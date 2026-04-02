from typing import Callable

from commands2 import TimedCommandRobot
from wpimath import units
import wpimath
import numpy

robot: TimedCommandRobot = None

def setRobotInstance(instance: TimedCommandRobot) -> None:
    global robot
    robot = instance

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
    robot.addPeriodic(callback, period, offset)

def apply_deadband(value: float) -> float:
    kDeadband = 0.05
    return wpimath.applyDeadband(value, kDeadband)

def apply_joystick_curves(input_vec: numpy.ndarray) -> numpy.ndarray:
    """
    Apply exponential curves to joystick input while maintaining direction.

    Args:
        input_vec: 2D numpy array representing joystick x and y input (values between -1 and 1)

    Returns:
        2D numpy array with curves applied, maintaining the input direction
    """
    kDeadband = 0.05
    # Get the magnitude of the vector (how far the joystick is pushed in that direction)
    mag = numpy.linalg.norm(input_vec)

    if mag <= kDeadband:
        # If the magnitude is very small, return zero vector (dead zone)
        return numpy.array([0.0, 0.0])

    # Cube the magnitude to provide finer control at low speeds while still allowing full speed at max joystick deflection
    # Maintain the direction of the input vector
    return input_vec * mag ** 2