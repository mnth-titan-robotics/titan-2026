from typing import Callable

from commands2 import TimedCommandRobot
from wpimath import units
import wpimath
robot: TimedCommandRobot = None

def setRobotInstance(instance: TimedCommandRobot) -> None:
    global robot
    robot = instance

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
    robot.addPeriodic(callback, period, offset)

def apply_deadband(value: float) -> float:
    kDeadband = 0.05
    return wpimath.applyDeadband(value, kDeadband)