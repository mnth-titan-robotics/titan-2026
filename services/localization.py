from wpilib import RobotBase
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
        self._posePublisher.set(Pose3d())
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
        prev_pose = self._pose
        # Store the frame for the next update
        self._pose = pose_frame.quest_pose_3d + self._questnavToRobot
        self._posePublisher.set(self._pose)
        if len(pose_frames) > 1:
            prev_frame = pose_frames[-2]
            prev_pose = prev_frame.quest_pose_3d + self._questnavToRobot
            dt = pose_frame.data_timestamp - prev_frame.data_timestamp
        else:
            dt = 0.02

        # Compute the velocity between the two frames
        delta = self._pose - prev_pose
        
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
        pose3d = Pose3d(
            pose.translation().x,
            pose.translation().y,
            self._pose.z,
            Rotation3d(0, 0, pose.rotation().radians()))
        if RobotBase.isSimulation():
            self._pose = pose3d
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
