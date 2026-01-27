#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from email import utils
from typing import Callable

from commands2 import Subsystem, Command
from ntcore import NetworkTableInstance
from wpilib import ADIS16470_IMU
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d, Transform2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Odometry, SwerveDrive4Kinematics

import constants
from services import Tracker
from .max_swerve_module import MAXSwerveModule

IMUAxis = ADIS16470_IMU.IMUAxis
DriveConstants = constants.Subsystems.Drive


class Drive(Subsystem):
  def __init__(self, tracker: Tracker):
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

    networkTable = NetworkTableInstance.getDefault()
    
    self._desiredStatePublisher = networkTable.getStructArrayTopic("Swerve/Modules/DesiredStates", SwerveModuleState).publish()
    self._statePublisher = networkTable.getStructArrayTopic("Swerve/Modules/States", SwerveModuleState).publish()
    self._posePublisher = networkTable.getStructTopic("Swerve/Pose", Pose2d).publish()
    self._anglePublisher = networkTable.getStructTopic("Swerve/Angle", Rotation2d).publish()

    # Odometry class for tracking robot pose
    self._odometry = SwerveDrive4Odometry(
      DriveConstants.kDriveKinematics,
      self._get_gyro_angle(),
      (
        self._frontLeft.getPosition(),
        self._frontRight.getPosition(),
        self._rearLeft.getPosition(),
        self._rearRight.getPosition()
      )
    )

  def periodic(self) -> None:
    self._odometry.update(
      self._get_gyro_angle(),
      (
        self._frontLeft.getPosition(),
        self._frontRight.getPosition(),
        self._rearLeft.getPosition(),
        self._rearRight.getPosition()
      )
    )
    self.updateTelemetry()

  def updateTelemetry(self) -> None:
    self._posePublisher.set(self.getPose())

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
    
    self._anglePublisher.set(self._get_gyro_angle())

  def getPose(self) -> Pose2d:
    """
    Returns the currently-estimated pose of the robot.
    :return: The pose.
    """
    return self._odometry.getPose()

  def resetOdometry(self, pose: Pose2d) -> None:
    """
    Resets the odometry to the specified pose.
    :param pose: The pose to which to set the odometry.
    """
    self._odometry.resetPosition(
      self._get_gyro_angle(),
      (
        self._frontLeft.getPosition(),
        self._frontRight.getPosition(),
        self._rearLeft.getPosition(),
        self._rearRight.getPosition()
      ),
      pose)
  
  def driveJoystickCommand(
      self,
      get_x: Callable[[], float],
      get_y: Callable[[], float],
      get_omega: Callable[[], float]) -> Command:
        """Returns a command that drives the robot with joystick input"""
        return self.driveCommand(
            lambda: ChassisSpeeds(
               get_x() * DriveConstants.kMaxSpeedMetersPerSecond,
               get_y() * DriveConstants.kMaxSpeedMetersPerSecond,
               get_omega() * DriveConstants.kMaxAngularSpeed
            )
        )
    
  def driveCommand(
            self,
            get_input: Callable[[], ChassisSpeeds]) -> Command:
        """Returns a command that drives the robot"""
        return self.run(
            lambda: self._set_chassis_speed(get_input())
        )

  def _set_chassis_speed(self, chassisSpeeds: ChassisSpeeds) -> None:
    swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)
    self._set_module_states(swerveModuleStates)

  def setX(self) -> None:
    """
    Sets the wheels into an X formation to prevent movement.
    """
    self._frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
    self._frontRight.setDesiredState((0, Rotation2d.fromDegrees(-45)))
    self._rearLeft.setDesiredState((0, Rotation2d.fromDegrees(-45)))
    self._rearRight.setDesiredState((0, Rotation2d.fromDegrees(45)))

  def _set_module_states(self, desiredStates: tuple[SwerveModuleState, ...]) -> None:
    """
    Sets the swerve ModuleStates.
    :param desiredStates: The desired SwerveModule states.
    """
    SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond)
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
  
  def _get_gyro_angle(self) -> Rotation2d:
    return Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ))

  def zeroHeading(self) -> None:
    """
    Zeroes the heading of the robot.
    """
    self._gyro.reset()

  def getHeading(self) -> float:
    """
    Returns the heading of the robot.
    :return: The robot's heading in degrees, from -180 to 180
    """
    return self._get_gyro_angle().degrees()

  def getTurnRate(self) -> float:
    """
    Returns the turn rate of the robot.
    :return: The turn rate of the robot, in degrees per second
    """
    multiplier = -1.0 if DriveConstants.kGyroReversed else 1.0
    return self._gyro.getRate(IMUAxis.kZ) * multiplier