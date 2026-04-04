#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from typing import Callable

from commands2 import Subsystem, Command, cmd
from ntcore import NetworkTableInstance
from wpilib import ADIS16470_IMU
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Translation2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Odometry, SwerveDrive4Kinematics
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
from wpimath import units
import wpimath
import math
import constants
from services import Tracker
from services.localization import Localization
from .max_swerve_module import MAXSwerveModule
import numpy
from lib.utils import apply_joystick_curves
from lib.gyro_navx2 import Gyro_NAVX2
from navx import AHRS
IMUAxis = ADIS16470_IMU.IMUAxis
DriveConstants = constants.Subsystems.Drive
ENABLE_TELEMETRY = constants.ENABLE_TELEMETRY


def getTargetHeading(sourcePose: Pose2d | Pose3d, targetPose: Pose2d | Pose3d, isRobotRelative: bool = False) -> units.degrees:
    if isinstance(sourcePose, Pose3d):
        sourcePose = sourcePose.toPose2d()
    if isinstance(targetPose, Pose3d):
        targetPose = targetPose.toPose2d()
    return math.atan2(targetPose.Y() - sourcePose.Y(), targetPose.X() - sourcePose.X()) - (sourcePose.rotation().radians() if isRobotRelative else 0)

class Drive(Subsystem):
    _fieldRelative = True
    _lockEnabled = False
    _lock_target = Pose2d()

    def __init__(self, localization: Localization, tracker: Tracker):
        self._localization = localization
        self._tracker = tracker
        # Create MAXSwerveModules
        self._frontLeft = MAXSwerveModule(
            DriveConstants.kFrontLeftDrivingCanId,
            DriveConstants.kFrontLeftTurningCanId,
            DriveConstants.kFrontLeftChassisAngularOffset
        )
        self._frontRight = MAXSwerveModule(
            DriveConstants.kFrontRightDrivingCanId,
            DriveConstants.kFrontRightTurningCanId,
            DriveConstants.kFrontRightChassisAngularOffset
        )
        self._rearLeft = MAXSwerveModule(
            DriveConstants.kRearLeftDrivingCanId,
            DriveConstants.kRearLeftTurningCanId,
            DriveConstants.kRearLeftChassisAngularOffset
        )
        self._rearRight = MAXSwerveModule(
            DriveConstants.kRearRightDrivingCanId,
            DriveConstants.kRearRightTurningCanId,
            DriveConstants.kRearRightChassisAngularOffset
        )

        # The gyro sensor
        self._gyro = ADIS16470_IMU()
        self._navX = Gyro_NAVX2(AHRS.NavXComType.kUSB1)
        self._theta_controller = ProfiledPIDControllerRadians(
            Kp=DriveConstants.RotationPID.P,
            Ki=DriveConstants.RotationPID.I,
            Kd=DriveConstants.RotationPID.D,
            constraints=TrapezoidProfileRadians.Constraints(
                maxVelocity=DriveConstants.kMaxSpeedMetersPerSecond,
                maxAcceleration=DriveConstants.kMaxAcceleration
            )
        )
        self._theta_controller.setTolerance(math.pi / 45.0)
        self._theta_controller.enableContinuousInput(-math.pi, math.pi)

        networkTable = NetworkTableInstance.getDefault()

        topic_key = "Swerve"
        self._fieldRelativePublisher = networkTable.getBooleanTopic(f"{topic_key}/FieldRelative").publish()
        if ENABLE_TELEMETRY:
            self._desiredStatePublisher = networkTable.getStructArrayTopic(
                "Swerve/Modules/DesiredStates", SwerveModuleState).publish()
            self._statePublisher = networkTable.getStructArrayTopic(
            f"{topic_key}/Modules/States", SwerveModuleState).publish()
            self._pose_publisher = networkTable.getStructTopic(f"{topic_key}/Pose", Pose2d).publish()
            self._localizationAnglePublisher = networkTable.getStructTopic(f"{topic_key}/QuestAngle", Rotation2d).publish()
            self._gyroAnglePublisher = networkTable.getStructTopic(f"{topic_key}/GyroAngle", Rotation2d).publish()
            self._target_publisher = networkTable.getStructTopic(f"{topic_key}/Target", Pose2d).publish()
            self._omega_publisher = networkTable.getFloatTopic(f"{topic_key}/Omega").publish()

        # Odometry class for tracking robot pose
        self._odometry = SwerveDrive4Odometry(
            DriveConstants.kDriveKinematics,
            self._get_localization_angle(),
            (
                self._frontLeft.getPosition(),
                self._frontRight.getPosition(),
                self._rearLeft.getPosition(),
                self._rearRight.getPosition()
            )
        )

    def periodic(self) -> None:
        self._odometry.update(
            self._get_localization_angle(),
            (
                self._frontLeft.getPosition(),
                self._frontRight.getPosition(),
                self._rearLeft.getPosition(),
                self._rearRight.getPosition()
            )
        )
        self._update_telemetry()
        self._update_lock_target()

    def _update_telemetry(self) -> None:
        self._fieldRelativePublisher.set(self._fieldRelative)
        if ENABLE_TELEMETRY:
            self._pose_publisher.set(self.get_pose())

            self._statePublisher.set([
                self._frontLeft.getState(),
                self._frontRight.getState(),
                self._rearLeft.getState(),
                self._rearRight.getState()]
            )

            self._desiredStatePublisher.set([
                self._frontLeft.getDesiredState(),
                self._frontRight.getDesiredState(),
                self._rearLeft.getDesiredState(),
                self._rearRight.getDesiredState()
            ])

            self._localizationAnglePublisher.set(self._get_localization_angle())
            self._gyroAnglePublisher.set(self._get_gyro_angle())
            self._target_publisher.set(self._lock_target)

    def toggle_lock_command(self, lock_target: Pose2d) -> Command:
        def run():
            self._lockEnabled = not self._lockEnabled
            self._lock_target = lock_target
            if self._lockEnabled:
                theta = getTargetHeading(self._localization.get_pose(), self._lock_target)
                self._theta_controller.reset(units.degreesToRadians(self._get_gyro_degrees()))
                self._theta_controller.setGoal(theta)
        return cmd.runOnce(run)

    def _update_lock_target(self):
        if self._lockEnabled:
            projected_pose = self._localization.get_projected_pose2d(DriveConstants.kTurnLatency)
            theta = getTargetHeading(projected_pose, self._lock_target)
            self._theta_controller.setGoal(theta)

    def get_pose(self) -> Pose2d:
        """
        Returns the currently-estimated pose of the robot.
        :return: The pose.
        """
        return self._odometry.getPose()

    def reset_odometry(self, pose: Pose2d) -> None:
        """
        Resets the odometry to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        self._odometry.resetPosition(
            self._get_localization_angle(),
            (
                self._frontLeft.getPosition(),
                self._frontRight.getPosition(),
                self._rearLeft.getPosition(),
                self._rearRight.getPosition()
            ),
            pose)

    def drive_joystick_command(
            self,
            get_x: Callable[[], float],
            get_y: Callable[[], float],
            get_omega: Callable[[], float]) -> Command:
        """Returns a command that drives the robot with joystick input"""
        def run() -> ChassisSpeeds:
            input_vec = numpy.array([get_x(), get_y()])
            omega = get_omega() * DriveConstants.kMaxAngularSpeed

            if ENABLE_TELEMETRY:
                self._omega_publisher.set(omega)

            # Apply joystick curves (deadzone and exponential)
            curved_input = apply_joystick_curves(input_vec)
            v = curved_input * DriveConstants.kMaxSpeedMetersPerSecond
            if self._fieldRelative:
                # Convert the input speeds from field-relative to robot-relative using the gyro angle
                output_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    v[0],
                    v[1],
                    omega,
                    self._get_localization_angle()
                )
            else:
                output_speeds = ChassisSpeeds(
                    v[0],
                    v[1],
                    omega
                )
            return output_speeds

        return self.drive_command(run)

    def drive_command(
            self,
            get_input: Callable[[], ChassisSpeeds]) -> Command:
        """Returns a command that drives the robot"""
        return self.run(
            lambda: self.set_chassis_speeds(get_input())
        )

    def toggle_field_relative_command(self) -> Command:
        """Returns a command that toggles field-relative control on and off"""
        def toggle():
            self._fieldRelative = not self._fieldRelative

        return self.runOnce(toggle).withName("ToggleFieldRelative")

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return DriveConstants.kDriveKinematics.toChassisSpeeds(self._get_module_states())  # type: ignore

    def set_chassis_speeds(self, chassisSpeeds: ChassisSpeeds) -> None:
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
        self._set_module_states(swerveModuleStates)

    def follow_trajectory_command(self, trajectory: Trajectory) -> Command:
        return cmd.none()

    def set_x_command(self) -> Command:
        """
        Sets the wheels into an X formation to prevent movement.
        """
        # TODO: Implement this command
        # self._frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        # self._frontRight.setDesiredState((0, Rotation2d.fromDegrees(-45)))
        # self._rearLeft.setDesiredState((0, Rotation2d.fromDegrees(-45)))
        # self._rearRight.setDesiredState((0, Rotation2d.fromDegrees(45)))
        return cmd.none()

    def alignToTargetPose(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
        return cmd.none()

    def _get_module_states(self) -> tuple[SwerveModuleState, ...]:
        return [
            self._frontLeft.getState(),
            self._frontRight.getState(),
            self._rearLeft.getState(),
            self._rearRight.getState()]  # type: ignore

    def _set_module_states(self, desiredStates: tuple[SwerveModuleState, ...]) -> None:
        """
        Sets the swerve ModuleStates.
        :param desiredStates: The desired SwerveModule states.
        """
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)  # type: ignore
        self._frontLeft.setDesiredState(desiredStates[0])
        self._frontRight.setDesiredState(desiredStates[1])
        self._rearLeft.setDesiredState(desiredStates[2])
        self._rearRight.setDesiredState(desiredStates[3])

    def _reset_encoders(self) -> None:
        """
        Resets the drive encoders to currently read a position of 0.
        """
        self._frontLeft.resetEncoders()
        self._rearLeft.resetEncoders()
        self._frontRight.resetEncoders()
        self._rearRight.resetEncoders()

    def _get_gyro_degrees(self) -> float:
        # return -self._gyro.getAngle(IMUAxis.kZ)
        return self._navX.getHeading()

    def _get_gyro_angle(self) -> Rotation2d:
        # Gyro is mounted upside-down, so we negate the angle
        return Rotation2d.fromDegrees(self._get_gyro_degrees())
    
    def _get_localization_angle(self) -> Rotation2d:
        # return self._localization.get_pose2d().rotation()
        return self._get_gyro_angle()

    def zeroHeading(self) -> None:
        """
        Zeroes the heading of the robot.
        """
        print('GYRO RESET')
        reset_pose = Pose2d(Translation2d(), Rotation2d())
        self._navX.resetRobotToField(reset_pose)
        self._gyro.reset()

    def getTurnRate(self) -> float:
        """
        Returns the turn rate of the robot.
        :return: The turn rate of the robot, in degrees per second
        """
        multiplier = -1.0 if DriveConstants.kGyroReversed else 1.0
        return self._gyro.getRate(IMUAxis.kZ) * multiplier
