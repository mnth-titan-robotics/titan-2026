from dataclasses import dataclass

from wpimath.geometry import Pose3d


@dataclass
class PoseFrame:
    """
    Represents a single frame of pose tracking data from the Quest headset.
    
    Mirrors the Java PoseFrame record from questnav-lib.
    
    Attributes:
        quest_pose_3d: The Quest's 3D pose in field coordinates
        data_timestamp: NetworkTables timestamp when data was received (use for pose estimator)
        app_timestamp: Quest app internal timestamp (for debugging only)
        frame_count: Sequential frame number from Quest
    """
    quest_pose_3d: Pose3d
    data_timestamp: float
    app_timestamp: float
    frame_count: int
