from typing import Callable
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
from wpimath import units
from dataclasses import dataclass, field
from lib import utils


@dataclass
class ShooterParams:
    """Class for keeping track of an item in inventory."""
    distance: float
    angle: float
    time_of_flight: float
    velocity: float = field(init=False)  # calculated in post_init

    def __post_init__(self) -> None:
        self.velocity = self.distance / self.time_of_flight


class TrackerConstants:
    kLatency = units.seconds(0.15)
    kParamLookup = [
        ShooterParams(1.5, 35.0, 0.38),
        ShooterParams(2.0, 40.0, 0.45),
        ShooterParams(2.5, 45.0, 0.52),
        ShooterParams(3.0, 50.0, 0.60),
        ShooterParams(3.5, 55.0, 0.68),
        ShooterParams(4.0, 58.0, 0.76),
        ShooterParams(4.5, 61.0, 0.85),
        ShooterParams(5.0, 64.0, 0.94),
    ]


class Tracker:
    """
    Class that handles automatically tracking/aiming toward a fixed pose.
    Based on the blog post by Eeshwar Krishnan - https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    """
    shot_angle: float = 0.0     # Angle relative to the field required to make the shot
    shot_distance: float = 0.0  # Distance to the target

    def __init__(
            self,
            constants: TrackerConstants,
            get_current_pose: Callable[[], Pose2d],
            get_velocity: Callable[[], ChassisSpeeds]):
        self._constants = constants
        self.enabled = False
        self._target_pose = Pose2d()
        self._absolute_heading = units.degrees(0)
        self._relative_heading = units.degrees(0)
        self._target_dist = units.meters(0)
        self.velocity = ChassisSpeeds()
        self._get_current_pose = get_current_pose
        self._get_velocity = get_velocity
        utils.addRobotPeriodic(self.update)

    def enable_tracking(self, target_pose: Pose2d):
        """Enables tracking toward the specified target pose."""
        self.enabled = True
        if self._target_pose != target_pose:
            self._target_pose = target_pose
            self.update()

    @property
    def target_dist(self) -> units.meters:
        return self._target_dist

    @property
    def absolute_heading(self) -> units.degrees:
        return self._absolute_heading

    @property
    def relative_heading(self) -> units.degrees:
        return self._relative_heading

    def update(self):
        if not self.enabled:
            return

        robot_pose = self._get_current_pose()
        (self._relative_heading, self._absolute_heading,
         self._target_dist) = utils.get_target_heading(robot_pose, self._target_pose)
