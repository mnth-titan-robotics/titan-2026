from typing import List, Optional

from hal import SimDevice
from wpimath.geometry import Pose3d, Rotation3d
import time
from .questnav_data import PoseFrame


class QuestNavStub:
    pose: Pose3d = Pose3d()
    battery_percent: float = 100.0
    tracking: bool = True
    connected: bool = True
    frame_count: int = 0
    tracking_lost_counter: int = 0
    latency: float = 0.0
    unread_frames: List[PoseFrame] = []
    
    def __init__(self):
        print("Creating STUB QuestNav Device")

    def get_all_unread_pose_frames(self) -> List[PoseFrame]:
        """
        Retrieves all new pose frames received since the last call.

        This is the primary method for integrating QuestNav with FRC pose
        estimation systems. Returns array of PoseFrame objects containing
        pose data and timestamps.

        Each frame contains:
        - Pose data: Quest position and orientation in field coordinates
        - NetworkTables timestamp: When data was received (use for pose estimation)
        - App timestamp: Quest internal timestamp (for debugging)
        - Frame count: Sequential frame number

        Returns:
            List of PoseFrame objects. Empty list if no new frames available.

        Example:
            frames = questnav.get_all_unread_pose_frames()
            for frame in frames:
                if questnav.is_tracking() and questnav.is_connected():
                    pose_estimator.add_vision_measurement(
                        frame.quest_pose_3d.toPose2d(),
                        frame.data_timestamp,
                        (0.1, 0.1, 0.05)  # Standard deviations
                    )
        """
        frames = self.unread_frames.copy()
        self.unread_frames.clear()
        return frames

    def set_pose(self, pose: Pose3d):
        self.pose = pose

    def get_battery_percent(self) -> Optional[int]:
        """
        Returns the Quest headset's current battery level as a percentage.

        Returns:
            Battery percentage (0-100), or None if no data available
        """
        return self.battery_percent

    def is_tracking(self) -> bool:
        """
        Gets the current tracking state of the Quest headset.

        Indicates whether the Quest's visual-inertial tracking system is
        currently functioning and providing reliable pose data.

        When tracking is lost, pose data becomes unreliable and should not
        be used for robot control.

        Returns:
            True if Quest is actively tracking, False if tracking is lost
            or no device data available
        """
        return self.tracking

    def is_connected(self) -> bool:
        """
        Determines if the Quest headset is currently connected.

        Connection is determined by how recent the last frame data was received.

        Returns:
            True if Quest is connected and sending data, False otherwise
        """
        return self.connected

    def is_reliable(self) -> bool:
        """True only when connected AND tracking. Mirrors QuestNav.is_reliable()."""
        return self.connected and self.tracking

    def get_frame_count(self) -> Optional[int]:
        """
        Gets the current frame count from the Quest headset.

        Returns:
            Frame count value, or None if no data available
        """
        return self.frame_count

    def get_tracking_lost_counter(self) -> Optional[int]:
        """
        Gets the number of tracking lost events since Quest connected.

        Returns:
            Tracking lost counter value, or None if no data available
        """
        return self.tracking_lost_counter

    def get_latency(self) -> float:
        """
        Gets the latency of the Quest to Robot connection.

        Returns latency between current time and last frame data update.

        Returns:
            Latency in milliseconds
        """
        current_time = time.time()
        return (current_time - self._last_frame_timestamp) * 1000.0

    def get_app_timestamp(self) -> Optional[float]:
        """
        Returns the Quest app's uptime timestamp.

        Important: For pose estimator integration, use the timestamp from
        PoseFrame.data_timestamp instead! This provides the Quest's internal
        timestamp for debugging only.

        Returns:
            Quest app uptime in seconds, or None if no data available
        """
        # This would need to be tracked from frame data
        # For now, return None
        return None

    def command_periodic(self):
        pass
