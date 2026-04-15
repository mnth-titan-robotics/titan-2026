'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

import numpy
from lib.utils import apply_joystick_curves
from pyfrc.tests import *
from target_heading_tests import *


def test_deadzone_at_origin():
    """Test that inputs at origin return zero"""
    result = apply_joystick_curves(numpy.array([0.0, 0.0]))
    numpy.testing.assert_array_equal(result, numpy.array([0.0, 0.0]))


def test_deadzone_within_threshold():
    """Test that small inputs within deadzone return zero"""
    # Test various inputs within 0.05 magnitude
    test_cases = [
        numpy.array([0.04, 0.0]),
        numpy.array([0.0, 0.04]),
        numpy.array([0.03, 0.03]),
        numpy.array([-0.04, 0.0]),
        numpy.array([0.0, -0.04]),
        numpy.array([-0.03, -0.03]),
    ]
    for input_vec in test_cases:
        result = apply_joystick_curves(input_vec)
        numpy.testing.assert_array_equal(result, numpy.array([0.0, 0.0]),
                                         err_msg=f"Input {input_vec} should be in deadzone")


def test_deadzone_boundary():
    """Test inputs at deadzone boundary (0.05)"""
    # Exactly at boundary should still be in deadzone
    result = apply_joystick_curves(numpy.array([0.05, 0.0]))
    numpy.testing.assert_array_equal(result, numpy.array([0.0, 0.0]))


def test_just_outside_deadzone():
    """Test that inputs just outside deadzone are not zeroed"""
    result = apply_joystick_curves(numpy.array([0.06, 0.0]))
    # Should not be zero
    assert not numpy.array_equal(result, numpy.array([0.0, 0.0]))


def test_exponential_curve_applied():
    """Test that exponential curve (magnitude cubed) is applied correctly"""
    # Test simple horizontal input
    input_vec = numpy.array([0.5, 0.0])
    result = apply_joystick_curves(input_vec)
    expected_magnitude = 0.5 ** 3  # mag^2 * original magnitude = mag^3
    expected = numpy.array([expected_magnitude, 0.0])
    numpy.testing.assert_array_almost_equal(result, expected, decimal=6)


def test_direction_maintained_positive_x():
    """Test that direction is maintained for positive x input"""
    input_vec = numpy.array([0.8, 0.0])
    result = apply_joystick_curves(input_vec)
    # Direction should be maintained (positive x)
    assert result[0] > 0
    assert abs(result[1]) < 1e-10


def test_direction_maintained_negative_x():
    """Test that direction is maintained for negative x input"""
    input_vec = numpy.array([-0.8, 0.0])
    result = apply_joystick_curves(input_vec)
    # Direction should be maintained (negative x)
    assert result[0] < 0
    assert abs(result[1]) < 1e-10


def test_direction_maintained_positive_y():
    """Test that direction is maintained for positive y input"""
    input_vec = numpy.array([0.0, 0.8])
    result = apply_joystick_curves(input_vec)
    # Direction should be maintained (positive y)
    assert abs(result[0]) < 1e-10
    assert result[1] > 0


def test_direction_maintained_negative_y():
    """Test that direction is maintained for negative y input"""
    input_vec = numpy.array([0.0, -0.8])
    result = apply_joystick_curves(input_vec)
    # Direction should be maintained (negative y)
    assert abs(result[0]) < 1e-10
    assert result[1] < 0


def test_direction_maintained_diagonal():
    """Test that direction is maintained for diagonal input"""
    # 45-degree angle
    input_vec = numpy.array([0.7071, 0.7071])  # ~1/sqrt(2)
    result = apply_joystick_curves(input_vec)

    # Both components should be positive
    assert result[0] > 0
    assert result[1] > 0

    # Ratio should be maintained (approximately 1:1)
    ratio = result[0] / result[1]
    assert abs(ratio - 1.0) < 0.01


def test_direction_maintained_various_angles():
    """Test that direction is maintained for various input angles"""
    test_angles = [30, 60, 120, 150, 210, 240, 300, 330]
    magnitude = 0.8

    for angle_deg in test_angles:
        angle_rad = numpy.radians(angle_deg)
        input_vec = numpy.array([
            magnitude * numpy.cos(angle_rad),
            magnitude * numpy.sin(angle_rad)
        ])
        result = apply_joystick_curves(input_vec)

        # Calculate angles
        input_angle = numpy.arctan2(input_vec[1], input_vec[0])
        result_angle = numpy.arctan2(result[1], result[0])

        # Angles should match
        assert abs(input_angle - result_angle) < 1e-5, \
            f"Direction not maintained for {angle_deg} degrees"


def test_full_deflection():
    """Test maximum joystick deflection"""
    input_vec = numpy.array([1.0, 0.0])
    result = apply_joystick_curves(input_vec)
    # mag = 1.0, so mag^2 = 1.0, result should be [1.0, 0.0]
    numpy.testing.assert_array_almost_equal(result, numpy.array([1.0, 0.0]))


def test_reduced_sensitivity_at_low_inputs():
    """Test that curve provides reduced sensitivity at low inputs"""
    # At 0.5 magnitude, output should be 0.125 (0.5^3)
    input_vec = numpy.array([0.5, 0.0])
    result = apply_joystick_curves(input_vec)
    assert abs(numpy.linalg.norm(result) - 0.125) < 1e-6

    # Output magnitude should be less than input magnitude (for inputs < 1)
    assert numpy.linalg.norm(result) < numpy.linalg.norm(input_vec)


def test_output_magnitude_scaling():
    """Test that output magnitude is input magnitude cubed"""
    test_magnitudes = [0.1, 0.3, 0.5, 0.7, 0.9, 1.0]

    for test_angle in range(0, 360, 30):
        angle_vec = numpy.array([numpy.cos(numpy.radians(test_angle)), numpy.sin(numpy.radians(test_angle))])
        for mag in test_magnitudes:
            if mag < 0.05:  # Skip deadzone
                continue
            input_vec = angle_vec * mag
            result = apply_joystick_curves(input_vec)
            expected_mag = mag ** 3
            actual_mag = numpy.linalg.norm(result)
            assert abs(actual_mag - expected_mag) < 1e-6, \
                f"Magnitude not cubed for input {mag}"
            numpy.testing.assert_array_almost_equal(result, angle_vec * expected_mag)
            print(f"Test angle: {test_angle}, input_vec: {input_vec}, output_vec: {result}")

