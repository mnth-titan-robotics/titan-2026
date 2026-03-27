import math
from typing import TYPE_CHECKING, Callable

from commands2 import Command, cmd
from commands2 import Command, cmd
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
from wpimath import units
from wpimath import units

import constants
from lib.differential_module import DifferentialControllerCommand

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


class Game:
    def __init__(self, robot: "RobotContainer"):
        self._robot = robot
        self._localization = robot.localization

    # TODO: Add any composite commands (anything that involves multiple steps or multiple subsystems) here
    # This is an example from The Lady Cans (FRC 2881):
    # def intakeCoralFromGround(self) -> Command:
    #   return (
    #     cmd.sequence(
    #       cmd.parallel(
    #         self._robot.elevator.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].elevator),
    #         self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].wrist).withTimeout(2.0),
    #         self._robot.arm.setPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].arm),
    #         self._robot.intake.intake()
    #       ).until(lambda: self.isIntakeHolding()),
    #       self.liftCoralFromIntake()
    #     )
    #     .onlyIf(lambda: not self.isGripperHolding())
    #     .withName("Game:IntakeCoralFromGround")
    #   )

    def followTrajectoryCommand(
            self,
            trajectory: Trajectory) -> Command:
        """Returns a command that drives the robot along the provided trajectory"""
        def get_pose() -> Pose2d:
            pose3d = self._localization.get_pose()
            rot2d = Rotation2d(pose3d.rotation().Z())
            return Pose2d(pose3d.X(), pose3d.Y(), rot2d)

        DriveConstants = constants.Subsystems.Drive
        driveSubsystem = self._robot.drive

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
            kinematics=DriveConstants.Mecanum.kDriveKinematics,
            controller=drive_controller,
            outputModuleStates=driveSubsystem._set_wheel_speeds,
            requirements=(driveSubsystem)
        )

    def driverResetCommand(self) -> Command:
        return cmd.parallel(
            cmd.runOnce(self._robot.drive.zeroHeading)
        )

    def operatorResetCommand(self) -> Command:
        return cmd.parallel(
            cmd.runOnce(self._robot.intakeExtender.resetEncoder)
        )
    
    def shakeIntakeCommand(self) -> Command:
        SHAKE_TIME = units.seconds(0.1)
        PAUSE_TIME = units.seconds(0.1)
        return cmd.repeatingSequence(
            self._robot.intakeExtender.retract().withTimeout(SHAKE_TIME),
            cmd.waitSeconds(PAUSE_TIME),
            self._robot.intakeExtender.extend().withTimeout(SHAKE_TIME),
            cmd.waitSeconds(PAUSE_TIME)
        )
    
    def agitateRobotCommand(self) -> Command:
        SHAKE_SPEED = 0.5
        SHAKE_TIME = 0.12
        forward = ChassisSpeeds(SHAKE_SPEED * constants.Subsystems.Drive.kMaxSpeedMetersPerSecond)
        reverse = ChassisSpeeds(SHAKE_SPEED * constants.Subsystems.Drive.kMaxSpeedMetersPerSecond)
        return cmd.repeatingSequence(
            self._robot.drive.drive_command(lambda:  forward).withTimeout(SHAKE_TIME),
            self._robot.drive.drive_command(lambda: reverse).withTimeout(SHAKE_TIME)
        )

    def pulseIndexerCommand(self) -> Command:
        """Runs the indexer, attempting to feed only one fuel at a time."""
        FEED_TIME = units.seconds(0.2)
        STOP_TIME = units.seconds(0.04)

        return cmd.repeatingSequence(
            self._robot.indexer.feed().withTimeout(FEED_TIME),
            self._robot.indexer.stop().withTimeout(STOP_TIME)
        )

    def feedFuelCommand(self) -> Command:
        """Runs the indexer while 'flapping' the intake"""
        return cmd.parallel(
            self.shakeIntakeCommand(),
            self.pulseIndexerCommand()
        )
    
    def feed_and_shoot(self):
        """Runs the launcher, then starts feeding fuel once the launcher is up to speed"""
        return cmd.parallel(
            self._robot.launcher.start(),
            cmd.none() \
                .until(self._robot.launcher.at_speed) \
                .andThen(cmd.parallel(
                    self.feedFuelCommand(),
                    self._robot.intake.intake_half_speed()
                    )
                )
        ) 

    def toggleLockOnHub(self) -> Command:
        return self._robot.drive.toggle_lock_command(constants.FieldConstants.kHubPose)