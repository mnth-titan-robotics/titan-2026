from typing import Callable
from wpimath.geometry import Pose2d, Rotation2d, Pose3d, Translation3d, Translation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds
from wpimath import units
from bisect import bisect_left
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
        self.target_location = Translation2d()
        self.velocity = ChassisSpeeds()
        self._get_current_pose = get_current_pose
        self._get_velocity = get_velocity
        utils.addRobotPeriodic(self.update)

    def enable_tracking(self, target_location: Translation2d):
        """Enables tracking toward the specified target pose."""
        self.target_location = target_location
        self.enabled = True

    def disable_tracking(self):
        """Disables tracking."""
        self.enabled = False

    def _interpolate(self, key: float, key_func: Callable[[ShooterParams], float]) -> ShooterParams:
        """Interpolates a value from the table of shooter parameters."""
        i = bisect_left(
            self._constants.kParamLookup,
            key,
            key=key_func)
        if i == 0:
            # Value is lower than any sampled parameter - return the min
            return self._constants.kParamLookup[0]
        if i == len(self._constants.kParamLookup):
            # Value is higher than any sampled parameter - return the max
            return self._constants.kParamLookup[-1]
        floor = self._constants.kParamLookup[i - 1]
        ceiling = self._constants.kParamLookup[i]
        # Key was exactly on a table value
        if key == key_func(ceiling):
            return ceiling
        f = key_func(floor)
        r = key_func(ceiling) - f
        t = (key - f) / r
        return ShooterParams(
            angle=floor.angle + (ceiling.angle - floor.angle) * t,
            time_of_flight=floor.time_of_flight + (ceiling.time_of_flight - floor.time_of_flight) * t,
            distance=floor.distance + (ceiling.distance - floor.distance) * t,
        )

    def _interpolate_dist(self, distance: float) -> ShooterParams:
        return self._interpolate(distance, lambda x: x.distance)

    def velocityToEffectiveDistance(self, velocity: float) -> float:
        params = self._interpolate(velocity, lambda x: x.velocity)
        return params.distance

    def calculateAdjustedRpm(self, requiredVelocity: float) -> float:
        params = self._interpolate(requiredVelocity, lambda x: x.velocity)
        return params.angle

    def update(self):
        if not self.enabled:
            return
        
        pose: Pose2d = self._get_current_pose()
        translation = pose.translation()
        v = self._get_velocity()

        robot_position = pose.translation()
        robot_velocity = Translation2d(v.vx, v.vy)
        goal_position = self.target_location
        latency_compensation = self._constants.kLatency

        # 1. Project future 
        future_pos = robot_position + (robot_velocity * latency_compensation)

        # 2. Get target vector
        to_goal = goal_position - future_pos
        distance = to_goal.norm()
        target_direction = to_goal / distance

        # 3. Look up baseline velocity from table
        baseline = self._interpolate_dist(distance)
        baseline_velocity = distance / baseline.time_of_flight

        # 4. Build target velocity vector
        target_velocity = target_direction * baseline_velocity

        # 5. THE MAGIC: subtract robot velocity
        shot_velocity = target_velocity - robot_velocity

        # 6. Extract results
        self.shot_angle = shot_velocity.angle()
        # robot_relative_angle = self.shot_angle - pose.rotation().radians()
        required_velocity = shot_velocity.norm()

        # 7. Use table in reverse: velocity → effective distance → angle
        effective_distance = self.velocityToEffectiveDistance(required_velocity)
        required_angle = self._interpolate(effective_distance, lambda x: x.distance).angle
