from wpimath.geometry import Pose2d, Pose3d, Transform3d, Translation3d, Rotation3d
from wpimath.kinematics import ChassisSpeeds
from .questnav import QuestNav
from lib import utils
from wpimath import units
import ntcore


class Localization:
    def __init__(self):
        self._questnav = QuestNav()
        self._pose = Pose3d()
        self._prev_frame = None
        self._velocity = ChassisSpeeds()
        utils.addRobotPeriodic(self.update)
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._posePublisher = self.nt_instance.getTable("Localization").getStructTopic("Pose3d", Pose3d).publish()
        self._velocityPublisher = self.nt_instance.getTable("Localization").getStructTopic("ChassisSpeeds", ChassisSpeeds).publish()
        self._robotToQuestnav = Transform3d(
            Translation3d(units.inchesToMeters(-9.0), units.inchesToMeters(13.5), units.inchesToMeters(14.125)),
            Rotation3d(0, 0, units.degreesToRadians(90))
        )
        self._questnavToRobot = self._robotToQuestnav.inverse()

    def update(self):
        self._questnav.command_periodic()
        pose_frames = self._questnav.get_all_unread_pose_frames()

        if not pose_frames:
            return

        pose_frame = pose_frames[-1]
        # print(f"pose_frame: data_timestamp={pose_frame.data_timestamp:.3f}, app_timestamp={pose_frame.app_timestamp:.3f}, frame_count={pose_frame.frame_count}")
        prev_frame = self._prev_frame
        # Store the frame for the next update
        self._pose = pose_frame.quest_pose_3d + self._questnavToRobot
        self._posePublisher.set(self._pose)
        self._prev_frame = pose_frame
        if len(pose_frames) > 1:
            prev_frame = pose_frames[-2]
            self._prev_frame = pose_frame
        elif prev_frame is None:
            return

        # Compute the velocity between the two frames
        delta = self._pose - prev_frame.quest_pose_3d
        dt = pose_frame.data_timestamp - prev_frame.data_timestamp
        if dt <= 0.001:
            return
        
        self._velocity = ChassisSpeeds(
            delta.x / dt,
            delta.y / dt,
            delta.rotation().angle / dt)
        self._velocityPublisher.set(self._velocity)

    def get_pose(self) -> Pose3d:
        return self._pose

    def get_pose2d(self) -> Pose2d:
        return self._pose.toPose2d()

    def reset_pose2d(self, pose: Pose2d) -> None:
        print(f"Resest pose: {pose.x}, {pose.y}")
        pose3d = Pose3d(
            pose.translation().x,
            pose.translation().y,
            self._pose.z,
            Rotation3d(0, 0, pose.rotation().radians()))
        self._questnav.set_pose(pose3d + self._questnavToRobot.inverse())

    def get_velocity(self) -> ChassisSpeeds:
        return self._velocity
