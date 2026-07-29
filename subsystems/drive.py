#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from typing import Callable

from commands2 import Subsystem, Command, cmd
from ntcore import NetworkTableInstance
from wpilib import ADIS16470_IMU
from wpimath import units
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Odometry, SwerveDrive4Kinematics
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
import math
import constants
from services import Tracker
from services.localization import Localization
from .max_swerve_module import MAXSwerveModule
from lib.utils import apply_joystick_curves, clamp
from lib.gyro_pigeon import Gyro_Pigeon2
IMUAxis = ADIS16470_IMU.IMUAxis
DriveConstants = constants.Subsystems.Drive
ENABLE_TELEMETRY = constants.ENABLE_TELEMETRY


class Drive(Subsystem):
    _fieldRelative = True
    _lockEnabled = False
    _lock_target = Pose2d()

    def __init__(self, localization: Localization, tracker: Tracker, gyro: Gyro_Pigeon2 | None = None):
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

        # The gyro sensor. Shared with Localization when supplied - see
        # RobotContainer._initServices.
        self._pigeon2 = gyro if gyro is not None else Gyro_Pigeon2()
        self._auto_aim_pid_controller = ProfiledPIDControllerRadians(
            Kp=DriveConstants.RotationPID.P,
            Ki=DriveConstants.RotationPID.I,
            Kd=DriveConstants.RotationPID.D,
            constraints=TrapezoidProfileRadians.Constraints(
                # ANGULAR constraints - this is a ProfiledPIDControllerRadians, so these
                # are rad/s and rad/s^2, not the linear drivetrain limits.
                maxVelocity=DriveConstants.kMaxAngularSpeed,
                maxAcceleration=DriveConstants.kMaxAngularAcceleration
            )
        )
        self._auto_aim_pid_controller.setTolerance(math.pi / 45.0)  # 2 degrees
        self._auto_aim_pid_controller.enableContinuousInput(-math.pi, math.pi)

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
            self._turn_rate_publisher = networkTable.getFloatTopic(f"{topic_key}/TurnRate").publish()
            self._lock_enabled_publisher = networkTable.getBooleanTopic(f"{topic_key}/LockEnabled").publish()

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

    def _start_lock(self) -> None:
        """Seeds the auto-aim controller for a fresh lock."""
        # Seed the motion profile at the CURRENT measurement, in radians.
        # Tracker.relative_heading is in degrees; feeding it raw to a
        # ProfiledPIDControllerRadians puts the profile's starting setpoint tens of
        # radians away from where the robot actually is, so the first thing the
        # profile does is drive the robot toward that bogus setpoint (away from the
        # target) before slowly walking the setpoint back in to the goal.
        self._auto_aim_pid_controller.reset(
            units.degreesToRadians(self._tracker.relative_heading))
        self._auto_aim_pid_controller.setGoal(0.0)

    def toggle_lock_command(self, lock_target: Pose2d) -> Command:
        def run():
            # Auto-aim needs a trustworthy position to compute the bearing to the target.
            # With QuestNav down we only have heading, so refuse to engage.
            if not self._lockEnabled and not self._localization.is_pose_reliable():
                self._lockEnabled = False
                return
            self._lockEnabled = not self._lockEnabled
            if self._lockEnabled:
                # TODO: add some way to let driver/operator to reset targeted hub
                #self._tracker.enable_tracking(lock_target)
                self._start_lock()
        return cmd.runOnce(run)

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
            turn_rate = get_omega() * DriveConstants.kMaxAngularSpeed

            # Drop the lock if localization degrades mid-lock - the bearing to the target is
            # computed from the robot's position, which is frozen once QuestNav drops out,
            # so the aim would silently walk off target. Hand rotation back to the driver.
            if self._lockEnabled and not self._localization.is_pose_reliable():
                self._lockEnabled = False

            if self._lockEnabled:
                # The profiled PID controller setpoint is 0 - this means positive relative angles give a negative result.
                # To compensate for this, negate the output of the PID controller.
                turn_rate = clamp(
                    -self._auto_aim_pid_controller.calculate(units.degreesToRadians(self._tracker.relative_heading)),
                    -DriveConstants.kMaxAngularSpeed,
                    DriveConstants.kMaxAngularSpeed)

            if ENABLE_TELEMETRY:
                self._turn_rate_publisher.set(turn_rate)
                self._lock_enabled_publisher.set(self._lockEnabled)

            # Apply joystick curves (deadzone and exponential)
            vx, vy = apply_joystick_curves(get_x(), get_y())
            vx *= DriveConstants.kMaxSpeedMetersPerSecond
            vy *= DriveConstants.kMaxSpeedMetersPerSecond
            if self._fieldRelative:
                # Convert the input speeds from field-relative to robot-relative using the gyro angle
                output_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx,
                    vy,
                    turn_rate,
                    self._get_localization_angle()
                )
            else:
                output_speeds = ChassisSpeeds(
                    vx,
                    vy,
                    turn_rate
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
        return self._pigeon2.getHeading()

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
        self._pigeon2.resetRobotToField(reset_pose)

    def getTurnRate(self) -> float:
        """
        Returns the turn rate of the robot.
        :return: The turn rate of the robot, in degrees per second
        """
        return self._pigeon2.getYawRate()
