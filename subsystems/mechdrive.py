from enum import Enum, auto
from typing import Callable

from commands2 import Subsystem, Command, cmd
from ntcore import NetworkTableInstance
from wpilib import ADIS16470_IMU
from wpilib import SendableChooser
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveWheelSpeeds, MecanumDriveWheelPositions, MecanumDriveOdometry
from wpimath.controller import HolonomicDriveController, ProfiledPIDControllerRadians
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians

import math
import constants
from lib.classes import MotorIdleMode, SpeedMode, DriveOrientation, OptionState
from lib.differential_module import DifferentialModule, DifferentialControllerCommand
from lib.enums import ModuleLocation
from services import Tracker
from services.localization import Localization

IMUAxis = ADIS16470_IMU.IMUAxis
DriveConstants = constants.Subsystems.Drive.Mecanum


class State(Enum):
    Disabled = auto()
    Enabled = auto()
    Stopped = auto()
    Running = auto()
    Completed = auto()


class Drive(Subsystem):
    _targetPose = Pose3d()
    _targetPoseAlignmentState = State.Disabled

    def __init__(self, localization: Localization, tracker: Tracker):
        self._localization = localization
        self._tracker = tracker
        super().__init__()

        self._gyro = ADIS16470_IMU()

        networkTable = NetworkTableInstance.getDefault()

        self._pose_publisher = networkTable.getStructTopic("Mecanum/Pose", Pose2d).publish()
        self._dx_publisher = networkTable.getDoubleTopic("Mecanum/Dx").publish()
        self._dy_publisher = networkTable.getDoubleTopic("Mecanum/Dy").publish()
        self._domega_publisher = networkTable.getDoubleTopic("Mecanum/Domega").publish()

        self._constants = DriveConstants

        self._differentialModules = dict((c.location, DifferentialModule(c))
                                         for c in self._constants.kDifferentialModuleConfigs)

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
        idleModeChooser.onChange(self._set_idle_mode)

        self._odometry = MecanumDriveOdometry(
            self._constants.kDriveKinematics,
            Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
            self._get_module_positions()
        )
        x_controller = PIDController(
            Kp=DriveConstants.TranslationPID.P,
            Ki=DriveConstants.TranslationPID.I,
            Kd=DriveConstants.TranslationPID.D)
        y_controller = PIDController(
            Kp=DriveConstants.TranslationPID.P + 0.2,
            Ki=DriveConstants.TranslationPID.I,
            Kd=DriveConstants.TranslationPID.D)
        theta_controller = ProfiledPIDControllerRadians(
            Kp=DriveConstants.RotationPID.P,
            Ki=DriveConstants.RotationPID.I,
            Kd=DriveConstants.RotationPID.D,
            constraints=TrapezoidProfileRadians.Constraints(
                maxVelocity=DriveConstants.kMaxSpeedMetersPerSecond,
                maxAcceleration=DriveConstants.kMaxSpeedMetersPerSecond
            )
        )
        # Allow the controller to wrap around. -180 degrees and 180 degrees (-math.pi and math.pi radians)
        # are the same.
        theta_controller.enableContinuousInput(-math.pi, math.pi)
        self._targetPoseAlignmentController = HolonomicDriveController(
            xController=x_controller,
            yController=y_controller,
            thetaController=theta_controller
        )

    def periodic(self) -> None:
        self._update_telemetry()

    def _update_telemetry(self) -> None:
        self._pose_publisher.set(self.get_pose())
        for module in self._differentialModules.values():
            module.updateTelemetry()

    def get_pose(self) -> Pose2d:
        """
        Returns the currently-estimated pose of the robot.
        :return: The pose.
        """
        return self._odometry.getPose()

    def reset_odometry(self, pose: Pose2d) -> None:
        self._odometry.resetPose(pose)

    def drive_joystick_command(
            self,
            get_x: Callable[[], float],
            get_y: Callable[[], float],
            get_omega: Callable[[], float]) -> Command:
        """Returns a command that drives the robot with joystick input"""
        def speeds_callback() -> ChassisSpeeds:
            dx = units.meters_per_second(get_x() * DriveConstants.kMaxSpeedMetersPerSecond)
            dy = units.meters_per_second(get_y() * DriveConstants.kMaxSpeedMetersPerSecond)
            domega = units.degrees_per_second(get_omega() * DriveConstants.kMaxAngularSpeed)
            return ChassisSpeeds(dx, dy, domega)
        return self.drive_command(speeds_callback)

    def drive_command(
            self,
            get_input: Callable[[], ChassisSpeeds]) -> Command:
        """Returns a command that drives the robot"""
        return self.run(
            lambda: self.set_chassis_speeds(get_input())
        ).withName("DriveSubsystem:Drive")

    def follow_trajectory_command(
            self,
            trajectory: Trajectory) -> Command:
        def get_pose() -> Pose2d:
            pose3d = self._localization.get_pose()
            rot2d = Rotation2d(pose3d.rotation().Z())
            return Pose2d(pose3d.X(), pose3d.Y(), rot2d)
        x_controller = PIDController(
            Kp=DriveConstants.TranslationPID.P,
            Ki=DriveConstants.TranslationPID.I,
            Kd=DriveConstants.TranslationPID.D)
        y_controller = PIDController(
            Kp=DriveConstants.TranslationPID.P + 0.2,
            Ki=DriveConstants.TranslationPID.I,
            Kd=DriveConstants.TranslationPID.D)
        theta_controller = ProfiledPIDControllerRadians(
            Kp=DriveConstants.RotationPID.P,
            Ki=DriveConstants.RotationPID.I,
            Kd=DriveConstants.RotationPID.D,
            constraints=TrapezoidProfileRadians.Constraints(
                maxVelocity=DriveConstants.kMaxSpeedMetersPerSecond,
                maxAcceleration=DriveConstants.kMaxAcceleration
            )
        )
        # Allow the controller to wrap around. -180 degrees and 180 degrees (-math.pi and math.pi radians)
        # are the same.
        theta_controller.enableContinuousInput(-math.pi, math.pi)
        drive_controller = HolonomicDriveController(
            xController=x_controller,
            yController=y_controller,
            thetaController=theta_controller
        )
        return DifferentialControllerCommand(
            trajectory=trajectory,
            pose=get_pose,
            kinematics=self._constants.kDriveKinematics,
            controller=drive_controller,
            outputModuleStates=self._set_wheel_speeds,
            requirements=tuple(self) # type: ignore
        )

    def set_x_command(self) -> Command:
        return cmd.none()

    def alignToTargetPose(self, getRobotPose: Callable[[], Pose2d], getTargetPose: Callable[[], Pose3d]) -> Command:
        return self.startRun(
            lambda: self._initTargetPoseAlignment(getTargetPose()),
            lambda: self._runTargetPoseAlignment(getRobotPose())
        ).until(
            lambda: self._targetPoseAlignmentState == State.Completed
        ).finallyDo(
            lambda end: self._endTargetPoseAlignment()
        )

    def _initTargetPoseAlignment(self, targetPose: Pose3d) -> None:
        self._targetPose = targetPose
        self._targetPoseAlignmentState = State.Running

    def _runTargetPoseAlignment(self, robotPose: Pose2d) -> None:
        targetPose = self._targetPose or Pose3d()
        self.set_chassis_speeds(self._targetPoseAlignmentController.calculate(
            robotPose, targetPose.toPose2d(), 0, targetPose.toPose2d().rotation()))
        if self._targetPoseAlignmentController.atReference():
            self._targetPoseAlignmentState = State.Completed

    def _endTargetPoseAlignment(self) -> None:
        self.set_chassis_speeds(ChassisSpeeds())
        if self._targetPoseAlignmentState != State.Completed:
            self._targetPoseAlignmentState = State.Stopped
            self._targetPose = None

    def set_chassis_speeds(self, chassisSpeeds: ChassisSpeeds) -> None:
        self._dx_publisher.set(chassisSpeeds.vx)
        self._dy_publisher.set(chassisSpeeds.vy)
        self._domega_publisher.set(chassisSpeeds.omega)
        wheel_speeds = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds)
        self._set_wheel_speeds(wheel_speeds)
        # TODO: Implement field-centric driving
        # self._drivetrain.driveCartesian(
        #     xSpeed=chassisSpeeds.vx,
        #     ySpeed=chassisSpeeds.vy,
        #     zRotation=chassisSpeeds.omega,
        #     gyroAngle=Rotation2d()
        # )

    def _set_wheel_speeds(self, wheelSpeeds: MecanumDriveWheelSpeeds) -> None:
        self._differentialModules[ModuleLocation.LeftFront].setVelocity(wheelSpeeds.frontLeft)
        self._differentialModules[ModuleLocation.RightFront].setVelocity(wheelSpeeds.frontRight)
        self._differentialModules[ModuleLocation.LeftRear].setVelocity(wheelSpeeds.rearLeft)
        self._differentialModules[ModuleLocation.RightRear].setVelocity(wheelSpeeds.rearRight)

    def _get_module_positions(self) -> MecanumDriveWheelPositions:
        wheel_positions = MecanumDriveWheelPositions()
        wheel_positions.frontLeft = self._differentialModules[ModuleLocation.LeftFront].getPosition()
        wheel_positions.frontRight = self._differentialModules[ModuleLocation.RightFront].getPosition()
        wheel_positions.rearLeft = self._differentialModules[ModuleLocation.LeftRear].getPosition()
        wheel_positions.rearRight = self._differentialModules[ModuleLocation.RightRear].getPosition()

        return wheel_positions

    def _get_wheel_speeds(self) -> MecanumDriveWheelSpeeds:
        wheel_speeds = MecanumDriveWheelSpeeds()
        wheel_speeds.frontLeft = self._differentialModules[ModuleLocation.LeftFront].getVelocity()
        wheel_speeds.frontRight = self._differentialModules[ModuleLocation.RightFront].getVelocity()
        wheel_speeds.rearLeft = self._differentialModules[ModuleLocation.LeftRear].getVelocity()
        wheel_speeds.rearRight = self._differentialModules[ModuleLocation.RightRear].getVelocity()
        return wheel_speeds

    def get_chassis_speeds(self) -> ChassisSpeeds:
        return self._constants.kDriveKinematics.toChassisSpeeds(self._get_wheel_speeds())

    def _set_idle_mode(self, idleMode: MotorIdleMode) -> None:
        # TODO: implement idleMode change on motor controllers
        # SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)
        pass
