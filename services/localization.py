from typing import TYPE_CHECKING, Optional

from wpilib import RobotBase, Timer
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Transform3d, Translation3d, Rotation3d
from wpimath.kinematics import ChassisSpeeds
from .questnav import QuestNav
from lib import utils
from wpimath import units
import ntcore

if TYPE_CHECKING:
    # Type-checking only: importing the gyro at runtime would pull constants.py (and
    # pathplannerlib) into every consumer of Localization. Any object exposing
    # getHeading()/getYawRate()/isConnected() works as a heading source.
    from lib.gyro_pigeon import Gyro_Pigeon2


class PoseSource:
    """Where the currently-published pose is coming from."""
    QUESTNAV = "QuestNav"
    GYRO_ONLY = "GyroOnly"   # QuestNav unavailable: heading is live, translation is frozen
    NONE = "None"            # Nothing trustworthy at all


class Localization:
    def __init__(self, gyro: 'Optional[Gyro_Pigeon2]' = None):
        self._questnav = QuestNav()
        self._gyro = gyro
        self._pose = Pose3d()
        self._velocity = ChassisSpeeds()
        utils.addRobotPeriodic(self.update)
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        _table = self.nt_instance.getTable("Localization")
        self._posePublisher = _table.getStructTopic("Pose3d", Pose3d).publish()
        self._posePublisher.set(Pose3d())
        self._velocityPublisher = _table.getStructTopic("ChassisSpeeds", ChassisSpeeds).publish()
        self._reliablePublisher = _table.getBooleanTopic("PoseReliable").publish()
        self._sourcePublisher = _table.getStringTopic("PoseSource").publish()
        self._questConnectedPublisher = _table.getBooleanTopic("QuestConnected").publish()
        self._questTrackingPublisher = _table.getBooleanTopic("QuestTracking").publish()
        self._robotToQuestnav = Transform3d(
            Translation3d(units.inchesToMeters(-9.0), units.inchesToMeters(13.5), units.inchesToMeters(14.125)),
            Rotation3d(0, 0, units.degreesToRadians(90))
        )
        self._questnavToRobot = self._robotToQuestnav.inverse()

        # Reliability / fallback state
        self._pose_reliable: bool = False
        self._source: str = PoseSource.NONE
        # Field yaw minus gyro yaw, captured while QuestNav is healthy. Applied to the raw
        # gyro reading during a dropout so the heading does not jump at the handoff.
        # Zero until the first good fix, in which case the raw gyro heading is used as-is.
        self._gyro_yaw_offset: units.degrees = 0.0
        self._last_reliable_timestamp: units.seconds = -1000.0

    def update(self):
        self._questnav.command_periodic()

        quest_connected = self._questnav.is_connected()
        quest_tracking = self._questnav.is_tracking()
        quest_reliable = quest_connected and quest_tracking

        if quest_reliable:
            self._update_from_questnav()
        else:
            self._update_from_gyro()

        self._publish_telemetry(quest_connected, quest_tracking)

    def _update_from_questnav(self) -> None:
        pose_frames = self._questnav.get_all_unread_pose_frames()

        if not pose_frames:
            # Connected and tracking, but nothing new this loop. Hold the last pose and
            # velocity rather than treating the gap as motion.
            return

        pose_frame = pose_frames[-1]
        prev_pose = self._pose
        self._pose = pose_frame.quest_pose_3d + self._questnavToRobot

        if len(pose_frames) > 1:
            prev_frame = pose_frames[-2]
            prev_pose = prev_frame.quest_pose_3d + self._questnavToRobot
            dt = pose_frame.data_timestamp - prev_frame.data_timestamp
        else:
            dt = 0.02

        self._pose_reliable = True
        self._source = PoseSource.QUESTNAV
        self._last_reliable_timestamp = Timer.getFPGATimestamp()
        self._capture_gyro_offset()

        if dt <= 0.001:
            return

        # Use the twist (log) rather than the raw pose difference. Pose3d.__sub__ gives a
        # Transform3d whose rotation().angle is the MAGNITUDE of the rotation vector and is
        # therefore never negative - that silently threw away the sign of omega. The twist
        # is signed and is also the exact inverse of the exp() used by
        # get_projected_pose2d(), so projection and measurement stay consistent.
        twist = prev_pose.log(self._pose)
        self._velocity = ChassisSpeeds(
            twist.dx / dt,
            twist.dy / dt,
            twist.rz / dt)

    def _update_from_gyro(self) -> None:
        """
        QuestNav is unavailable. Keep publishing a live heading from the pigeon and freeze
        the translation at the last trusted position.

        Wheel-odometry translation is deliberately out of scope here - the X/Y in this pose
        is stale and callers must check is_pose_reliable() before using it for anything
        that depends on position (range to target, auto-align, shoot-on-the-fly).
        """
        self._pose_reliable = False

        if self._gyro is None:
            self._source = PoseSource.NONE
            self._velocity = ChassisSpeeds()
            return

        self._source = PoseSource.GYRO_ONLY
        heading = utils.wrapAngle(self._gyro.getHeading() + self._gyro_yaw_offset)
        self._pose = Pose3d(
            self._pose.translation(),
            Rotation3d(0.0, 0.0, units.degreesToRadians(heading))
        )
        # We can still measure rotation, but translation velocity is unknowable without
        # QuestNav, so report zero rather than a stale value that would feed bad lead
        # compensation into shoot-on-the-fly.
        self._velocity = ChassisSpeeds(
            0.0, 0.0, units.degreesToRadians(self._gyro.getYawRate()))

    def _capture_gyro_offset(self) -> None:
        """
        Records the difference between the field-frame yaw and the raw gyro yaw while
        QuestNav is healthy, so that a dropout does not produce a heading discontinuity.
        """
        if self._gyro is None:
            return
        self._gyro_yaw_offset = utils.wrapAngle(
            self._pose.toPose2d().rotation().degrees() - self._gyro.getHeading())

    def _publish_telemetry(self, quest_connected: bool, quest_tracking: bool) -> None:
        self._posePublisher.set(self._pose)
        self._velocityPublisher.set(self._velocity)
        self._reliablePublisher.set(self._pose_reliable)
        self._sourcePublisher.set(self._source)
        self._questConnectedPublisher.set(quest_connected)
        self._questTrackingPublisher.set(quest_tracking)

    def is_pose_reliable(self) -> bool:
        """
        True when the published pose translation can be trusted.

        False means QuestNav is disconnected or has lost tracking: rotation() is still live
        (from the pigeon) but the translation is frozen at its last known value.
        """
        return self._pose_reliable

    def is_heading_reliable(self) -> bool:
        """Heading stays usable through a QuestNav dropout as long as the pigeon is alive."""
        if self._pose_reliable:
            return True
        return self._gyro is not None and self._gyro.isConnected()

    def get_pose_source(self) -> str:
        return self._source

    def get_time_since_reliable(self) -> units.seconds:
        """Seconds since QuestNav last gave us a trustworthy pose."""
        return Timer.getFPGATimestamp() - self._last_reliable_timestamp

    def get_pose(self) -> Pose3d:
        return self._pose

    def get_pose2d(self) -> Pose2d:
        return self._pose.toPose2d()

    def get_heading(self) -> Rotation2d:
        """Field-relative heading. Valid through a QuestNav dropout."""
        return self._pose.toPose2d().rotation()

    def reset_pose2d(self, pose: Pose2d) -> None:
        pose3d = Pose3d(
            pose.translation().x,
            pose.translation().y,
            self._pose.z,
            Rotation3d(0, 0, pose.rotation().radians()))
        self._pose = pose3d
        self._capture_gyro_offset()
        if RobotBase.isSimulation():
            self._posePublisher.set(self._pose)
        else:
            self._questnav.set_pose(pose3d + self._questnavToRobot.inverse())

    def get_velocity(self) -> ChassisSpeeds:
        return self._velocity

    def get_projected_pose2d(self, dt: float) -> Pose2d:
        cur_pose = self._pose.toPose2d()
        velocity = self._velocity
        twist = velocity.toTwist2d(dt)
        return cur_pose.exp(twist)
