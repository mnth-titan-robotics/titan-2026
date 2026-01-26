from wpimath.geometry import Pose3d
from wpimath.kinematics import ChassisSpeeds
from .questnav import QuestNav
from lib import utils
from wpimath import units


class Localization:
    def __init__(self):
        self._questnav = QuestNav()
        self._pose = Pose3d()
        self._prev_frame = None
        self._velocity = ChassisSpeeds()
        utils.addRobotPeriodic(self.update)

    def update(self):
        self._questnav.command_periodic()
        pose_frames = self._questnav.get_all_unread_pose_frames()

        if not pose_frames:
            return

        pose_frame = pose_frames[-1]
        prev_frame = self._prev_frame
        # Store the frame for the next update
        self._pose = pose_frame.quest_pose_3d
        self._prev_frame = pose_frame
        if len(pose_frames) > 1:
            prev_frame = pose_frames[-2]
            self._prev_frame = pose_frame

        if not prev_frame:
            return

        # Compute the velocity between the two frames
        delta = self._pose - prev_frame.quest_pose_3d
        dt = pose_frame.data_timestamp - prev_frame.data_timestamp
        self._velocity = ChassisSpeeds(
            delta.x / dt,
            delta.y / dt,
            delta.rotation().angle / dt)

    def get_pose(self) -> Pose3d:
        return self._pose

    def get_velocity(self) -> ChassisSpeeds:
        return self._velocity
