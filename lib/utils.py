from typing import Callable

from commands2 import TimedCommandRobot
from wpimath import units

robot: TimedCommandRobot = None

def setRobotInstance(instance: TimedCommandRobot) -> None:
    global robot
    robot = instance

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
    robot.addPeriodic(callback, period, offset)