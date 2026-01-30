from typing import Callable

from commands2 import Subsystem, Command
from ntcore import NetworkTableInstance
from wpilib import ADIS16470_IMU
from wpilib import SendableChooser
from wpilib.drive import MecanumDrive
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveWheelSpeeds, MecanumDriveWheelPositions
from wpimath.kinematics import MecanumDriveOdometry

import constants
from lib.classes import MotorIdleMode, SpeedMode, DriveOrientation, OptionState
from lib.differential_module import DifferentialModule
from lib.enums import ModuleLocation

IMUAxis = ADIS16470_IMU.IMUAxis
DriveConstants = constants.Subsystems.Drive


class Drive(Subsystem):
    def __init__(self):
        super().__init__()

        self._gyro = ADIS16470_IMU()

        networkTable = NetworkTableInstance.getDefault()

        self._publisher = networkTable.getStructTopic("Mecanum/Pose", Pose2d).publish()

        self._constants = constants.Subsystems.Drive.Mecanum

        self._differentialModules = dict((c.location, DifferentialModule(c)) for c in self._constants.kDifferentialModuleConfigs)

        self._drivetrain = MecanumDrive(
            self._differentialModules[ModuleLocation.LeftFront].getMotorController(),
            self._differentialModules[ModuleLocation.LeftRear].getMotorController(),
            self._differentialModules[ModuleLocation.RightFront].getMotorController(),
            self._differentialModules[ModuleLocation.RightRear].getMotorController()
        )

        self._isDriftCorrectionActive: bool = False
        self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
        self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
        self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

        self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
        self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
        self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

        self._speedMode: SpeedMode = SpeedMode.Competition
        speedModeChooser = SendableChooser()
        speedModeChooser.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
        speedModeChooser.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
        speedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
        # SmartDashboard.putData("Robot/Drive/SpeedMode", speedModeChooser)

        self._orientation: DriveOrientation = DriveOrientation.Field
        orientationChooser = SendableChooser()
        orientationChooser.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
        orientationChooser.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
        orientationChooser.onChange(lambda orientation: setattr(self, "_orientation", orientation))
        # SmartDashboard.putData("Robot/Drive/Orientation", orientationChooser)

        self._driftCorrection: OptionState = OptionState.Enabled
        driftCorrectionChooser = SendableChooser()
        driftCorrectionChooser.setDefaultOption(OptionState.Enabled.name, OptionState.Enabled)
        driftCorrectionChooser.addOption(OptionState.Disabled.name, OptionState.Disabled)
        driftCorrectionChooser.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
        # SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrectionChooser)

        idleModeChooser = SendableChooser()
        idleModeChooser.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
        idleModeChooser.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
        idleModeChooser.onChange(lambda idleMode: self._setIdleMode(idleMode))

        self._odometry = MecanumDriveOdometry(
            self._constants.kDriveKinematics,
            Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
            self._getModulePositions()
        )

    def periodic(self) -> None:
        self._updateTelemetry()

    def _updateTelemetry(self) -> None:
        self._publisher.set(self.getPose())

    def getPose(self) -> Pose2d:
        """
        Returns the currently-estimated pose of the robot.
        :return: The pose.
        """
        return self._odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        self._odometry.resetPose(pose)

    def driveCommand(
            self,
            get_input: Callable[[], ChassisSpeeds]) -> Command:
        """Returns a command that drives the robot"""
        return self.run(
            lambda: self.setChassisSpeed(get_input())
        ).withName("DriveSubsystem:Drive")

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool) -> None:
        pass

    def setChassisSpeed(self, chassisSpeeds: ChassisSpeeds) -> None:
        self._drivetrain.driveCartesian(
            xSpeed=chassisSpeeds.vx,
            ySpeed=chassisSpeeds.vy,
            zRotation=chassisSpeeds.omega,
            gyroAngle=Rotation2d()
        )

    def _getModulePositions(self) -> MecanumDriveWheelPositions:
        wheel_positions = MecanumDriveWheelPositions()
        wheel_positions.frontLeft = self._differentialModules[ModuleLocation.LeftFront].getPosition()
        wheel_positions.frontRight = self._differentialModules[ModuleLocation.RightFront].getPosition()
        wheel_positions.rearLeft = self._differentialModules[ModuleLocation.LeftRear].getPosition()
        wheel_positions.rearRight = self._differentialModules[ModuleLocation.RightRear].getPosition()

        return wheel_positions

    def getChassisSpeeds(self) -> ChassisSpeeds:
        wheel_velocities = MecanumDriveWheelSpeeds()
        wheel_velocities.frontLeft = self._differentialModules[ModuleLocation.LeftFront].getVelocity()
        wheel_velocities.frontRight = self._differentialModules[ModuleLocation.RightFront].getVelocity()
        wheel_velocities.rearLeft = self._differentialModules[ModuleLocation.LeftRear].getVelocity()
        wheel_velocities.rearRight = self._differentialModules[ModuleLocation.RightRear].getVelocity()
        return self._constants.kDriveKinematics.toChassisSpeeds(wheel_velocities)

    def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
        # TODO: implement idleMode change on motor controllers
        # SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)
        pass

    def setX(self) -> None:
        pass
