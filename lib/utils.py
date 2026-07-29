from typing import Callable, TypeVar

from commands2 import TimedCommandRobot
from wpilib import DriverStation
from wpimath import units
import wpimath
from wpimath.geometry import Pose2d, Pose3d
import math

Alliance = DriverStation.Alliance
T = TypeVar("T")
robot: TimedCommandRobot = None # type: ignore


def setRobotInstance(instance: TimedCommandRobot) -> None:
    global robot
    robot = instance

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamps a value between a minimum and maximum."""
    return max(min_value, min(max_value, value))

def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
    robot.addPeriodic(callback, period, offset)


def getAlliance() -> Alliance:
    return DriverStation.getAlliance() or Alliance.kBlue


def getValueForAlliance(blueValue: T, redValue: T) -> T:
    return blueValue if DriverStation.getAlliance() == Alliance.kBlue else redValue


def apply_deadband(value: float) -> float:
    kDeadband = 0.05
    return wpimath.applyDeadband(value, kDeadband)


def wrapAngle(angle: units.degrees) -> units.degrees:
    return wpimath.inputModulus(angle, -180, 180)


def apply_joystick_curves(x: float, y: float) -> tuple[float, float]:
    """
    Apply exponential curves to joystick input while maintaining direction.

    These run every loop, so this is deliberately plain scalar math - allocating a
    2-element numpy array and calling linalg.norm on it costs several times more than
    the arithmetic it replaces.

    Args:
        x: joystick x input (-1 to 1)
        y: joystick y input (-1 to 1)

    Returns:
        (x, y) with curves applied, maintaining the input direction
    """
    kDeadband = 0.05
    # Get the magnitude of the vector (how far the joystick is pushed in that direction)
    mag = math.hypot(x, y)

    if mag <= kDeadband:
        # If the magnitude is very small, return zero vector (dead zone)
        return 0.0, 0.0

    # Cube the magnitude to provide finer control at low speeds while still allowing full
    # speed at max joystick deflection. Maintain the direction of the input vector.
    scale = mag * mag
    return x * scale, y * scale


def get_target_heading(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d) -> tuple[units.degrees, units.degrees, units.meters]:
    if isinstance(sourcePose, Pose3d):
        sourcePose = sourcePose.toPose2d()
    if isinstance(targetPose, Pose3d):
        targetPose = targetPose.toPose2d()

    # Called every loop from Tracker.update() - plain scalar math, no numpy allocations.
    dx = targetPose.X() - sourcePose.X()
    dy = targetPose.Y() - sourcePose.Y()
    distance = math.hypot(dx, dy)
    absolute_angle = math.degrees(math.atan2(dy, dx))
    relative_angle = wrapAngle(absolute_angle - sourcePose.rotation().degrees())
    return relative_angle, absolute_angle, units.meters(distance)
