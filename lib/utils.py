from typing import Callable

from commands2 import TimedCommandRobot
from wpimath import units
import wpimath
from wpimath.geometry import Pose2d, Pose3d
import numpy
import math

robot: TimedCommandRobot = None


def setRobotInstance(instance: TimedCommandRobot) -> None:
    global robot
    robot = instance


def addRobotPeriodic(callback: Callable[[], None], period: units.seconds = 0.02, offset: units.seconds = 0) -> None:
    robot.addPeriodic(callback, period, offset)


def apply_deadband(value: float) -> float:
    kDeadband = 0.05
    return wpimath.applyDeadband(value, kDeadband)


def wrapAngle(angle: units.degrees) -> units.degrees:
  return wpimath.inputModulus(angle, -180, 180)


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


def get_target_heading(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d, isRobotRelative: bool = False) -> tuple[units.degrees, units.meters]:
    if isinstance(sourcePose, Pose3d):
        sourcePose = sourcePose.toPose2d()
    if isinstance(targetPose, Pose3d):
        targetPose = targetPose.toPose2d()

    source_vec = numpy.array([sourcePose.X(), sourcePose.Y()])
    target_vec = numpy.array([targetPose.X(), targetPose.Y()])
    to_target = target_vec - source_vec
    distance = numpy.linalg.norm(to_target)
    angle = math.atan2(to_target[1], to_target[0])
    if isRobotRelative:
        angle -= sourcePose.rotation().radians()
    return units.radiansToDegrees(angle), units.meters(distance)
