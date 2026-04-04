import math
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d
from lib.utils import get_target_heading

def test_get_target_heading_basic_field_relative():
    """Test get_target_heading with basic field-relative positions"""
    # Robot at origin, target at (1, 0)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose2d(1, 0, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(angle - 0) < 1e-6, f"Expected 0 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_90_degrees_field_relative():
    """Test get_target_heading at 90 degrees field-relative"""
    # Robot at origin, target at (0, 1)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose2d(0, 1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(angle - 90) < 1e-6, f"Expected 90 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_180_degrees_field_relative():
    """Test get_target_heading at 180 degrees field-relative"""
    # Robot at origin, target at (-1, 0)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose2d(-1, 0, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(abs(angle) - 180) < 1e-6, f"Expected ±180 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_negative_90_degrees_field_relative():
    """Test get_target_heading at -90 degrees field-relative"""
    # Robot at origin, target at (0, -1)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose2d(0, -1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(angle - (-90)) < 1e-6, f"Expected -90 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_diagonal_field_relative():
    """Test get_target_heading with diagonal target"""
    # Robot at origin, target at (1, 1)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose2d(1, 1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(angle - 45) < 1e-6, f"Expected 45 degrees, got {angle}"
    expected_distance = math.sqrt(2)
    assert abs(distance - expected_distance) < 1e-6, f"Expected {expected_distance} meters, got {distance}"


def test_get_target_heading_robot_relative_same_heading():
    """Test robot-relative angle when robot faces same direction as target"""
    # Robot at origin facing 0 degrees, target at (1, 0)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose2d(1, 0, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=True)

    assert abs(angle - 0) < 1e-6, f"Expected 0 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_robot_relative_90_degree_offset():
    """Test robot-relative angle when robot is rotated 90 degrees"""
    # Robot at origin facing 90 degrees, target at (1, 0)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(90))
    target = Pose2d(1, 0, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=True)

    # Field angle to target is 0, robot is facing 90, so robot-relative is -90
    assert abs(angle - (-90)) < 1e-6, f"Expected -90 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_robot_relative_180_degree_offset():
    """Test robot-relative angle when robot is facing opposite direction"""
    # Robot at origin facing 180 degrees, target at (1, 0)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(180))
    target = Pose2d(1, 0, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=True)

    # Field angle to target is 0, robot is facing 180, so robot-relative is -180
    assert abs(abs(angle) - 180) < 1e-6, f"Expected ±180 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_robot_relative_arbitrary_rotation():
    """Test robot-relative angle with arbitrary robot rotation"""
    # Robot at origin facing 45 degrees, target at (1, 1)
    source = Pose2d(0, 0, Rotation2d.fromDegrees(45))
    target = Pose2d(1, 1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=True)

    # Field angle to target is 45, robot is facing 45, so robot-relative is 0
    assert abs(angle - 0) < 1e-6, f"Expected 0 degrees, got {angle}"
    expected_distance = math.sqrt(2)
    assert abs(distance - expected_distance) < 1e-6, f"Expected {expected_distance} meters, got {distance}"


def test_get_target_heading_non_origin_source():
    """Test get_target_heading with source not at origin"""
    # Robot at (2, 3), target at (5, 7)
    source = Pose2d(2, 3, Rotation2d.fromDegrees(0))
    target = Pose2d(5, 7, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    # Delta: (3, 4) -> atan2(4, 3) = 53.13 degrees
    expected_angle = math.degrees(math.atan2(4, 3))
    expected_distance = 5.0  # 3-4-5 triangle

    assert abs(angle - expected_angle) < 1e-6, f"Expected {expected_angle} degrees, got {angle}"
    assert abs(distance - expected_distance) < 1e-6, f"Expected {expected_distance} meters, got {distance}"


def test_get_target_heading_zero_distance():
    """Test get_target_heading when source and target are the same"""
    # Robot and target at same position
    source = Pose2d(1, 1, Rotation2d.fromDegrees(0))
    target = Pose2d(1, 1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(distance) < 1e-6, f"Expected 0 meters, got {distance}"
    # Angle is undefined when distance is 0, but should still return a value


def test_get_target_heading_pose3d_to_pose2d_conversion():
    """Test that Pose3d inputs are correctly converted to Pose2d"""
    # Use Pose3d for source and target
    source = Pose3d(0, 0, 0, Rotation3d(0, 0, 0))
    target = Pose3d(1, 1, 5, Rotation3d(0, 0, 0))  # Z coordinate should be ignored
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    # Should only consider X and Y
    assert abs(angle - 45) < 1e-6, f"Expected 45 degrees, got {angle}"
    expected_distance = math.sqrt(2)
    assert abs(distance - expected_distance) < 1e-6, f"Expected {expected_distance} meters, got {distance}"


def test_get_target_heading_mixed_pose_types():
    """Test with mixed Pose2d and Pose3d inputs"""
    # Source as Pose2d, target as Pose3d
    source = Pose2d(0, 0, Rotation2d.fromDegrees(0))
    target = Pose3d(1, 0, 10, Rotation3d(0, 0, 0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(angle - 0) < 1e-6, f"Expected 0 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"

    # Source as Pose3d, target as Pose2d
    source = Pose3d(0, 0, 5, Rotation3d(0, 0, 0))
    target = Pose2d(0, 1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    assert abs(angle - 90) < 1e-6, f"Expected 90 degrees, got {angle}"
    assert abs(distance - 1.0) < 1e-6, f"Expected 1.0 meters, got {distance}"


def test_get_target_heading_negative_coordinates():
    """Test get_target_heading with negative coordinates"""
    # All quadrants
    source = Pose2d(-2, -3, Rotation2d.fromDegrees(0))
    target = Pose2d(1, 1, Rotation2d.fromDegrees(0))
    angle, distance = get_target_heading(source, target, isRobotRelative=False)

    # Delta: (3, 4)
    expected_angle = math.degrees(math.atan2(4, 3))
    expected_distance = 5.0

    assert abs(angle - expected_angle) < 1e-6, f"Expected {expected_angle} degrees, got {angle}"
    assert abs(distance - expected_distance) < 1e-6, f"Expected {expected_distance} meters, got {distance}"

