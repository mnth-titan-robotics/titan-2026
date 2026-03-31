from typing import TYPE_CHECKING

from commands2 import Command, cmd

from wpilib import DriverStation, SmartDashboard
from wpimath import units
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import TrajectoryGenerator, TrajectoryConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from commands.game import Game
from pathplannerlib.events import EventTrigger
from pathplannerlib.auto import AutoBuilder, DriveFeedforwards, NamedCommands, PathPlannerAuto, FlippingUtil
from pathplannerlib.controller import PPHolonomicDriveController
import constants

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


def shouldFlipPath():
    # Boolean supplier that controls when the path will be mirrored for the red alliance
    # This will flip the path being followed to the red side of the field.
    # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
    return DriverStation.getAlliance() == DriverStation.Alliance.kRed


class Auto:
    def __init__(
            self,
            robot: "RobotContainer",
            game: Game
    ) -> None:
        self._robot = robot
        self._game = game

        self._selectedAuto = cmd.none()
        DriveConstants = constants.Subsystems.Drive
        
        NamedCommands.registerCommand('climbtower', cmd.none())
        NamedCommands.registerCommand('startLauncher', self._game.feed_and_shoot())
        NamedCommands.registerCommand('runLauncher', self._game.feed_and_shoot())
        NamedCommands.registerCommand('lowerIntake', self._robot.intakeExtender.auto_extend())
        NamedCommands.registerCommand('runIntake', self._robot.intake.intake())

        def output(chassisSpeeds: ChassisSpeeds, driveFeedForward: DriveFeedforwards) -> None:
            self._robot.drive.set_chassis_speeds(chassisSpeeds)

        AutoBuilder.configure(
            self._robot.localization.get_pose2d,
            self._robot.localization.reset_pose2d,
            self._robot.drive.get_chassis_speeds,
            output,
            PPHolonomicDriveController(DriveConstants.kPathPlannerTranslationConstants,
                                       DriveConstants.kPathPlannerRotationConstants),
            DriveConstants.PathPlannerConfig,
            shouldFlipPath,
            self._robot.drive
        )

        # Add Robot/Auto to SmartDashboard with options
        # Default 'None' auto, does nothing. The second parameter is a function that will be called in our onChange
        # callback below
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Robot/Auto", self.autoChooser)
        self.autoChooser.onChange(self._onAutoChange)
        
    def _onAutoChange(self, selected_auto) -> None:
        if isinstance(selected_auto, PathPlannerAuto):
            starting_pose = selected_auto._startingPose
            if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
                starting_pose = FlippingUtil.flipFieldPose(starting_pose)
            self._robot.localization.reset_pose2d(starting_pose)


    def get(self):
        return self.autoChooser.getSelected()

    def set(self, autoCmd: Command) -> None:
        self._selectedAuto = autoCmd

    def auto_center(self) -> Command:
        # Move backward at 75% speed for 0.5s, launch fuel for 5s, then stop
        drive = self._robot.drive
        speeds = ChassisSpeeds(vx=-0.75)
        return cmd.sequence(
            
            drive.drive_command(lambda: speeds).withTimeout(0.5),
            self.auto_launch(units.seconds(2.0))
        )

    def auto_test(self) -> Command:
        # A simple test auto that spins in place for 2 seconds

        trajectory = TrajectoryGenerator.generateTrajectory(
            Pose2d(0, 0, Rotation2d(0)),
            [Translation2d(1, 0), Translation2d(0, 1), Translation2d(1.2, 1.2)],
            Pose2d(1, 1, Rotation2d(0)),
            TrajectoryConfig(
                units.meters_per_second(1.0),
                units.meters_per_second_squared(1.5)
            )
        )

        return self._game.followTrajectoryCommand(
            trajectory=trajectory
        )
    
    def _feed_when_at_speed(self) -> Command:
        return self._robot.indexer.stop() \
            .until(self._robot.launcher.at_speed) \
            .andThen(self._robot.indexer.feed())

    def auto_launch(self, duration: units.seconds) -> Command:
        # Auto command that starts the launcher, waits until at speed, then launches for the specified duration
        return cmd.parallel(
            self._robot.launcher.start(),
            self._feed_when_at_speed()
        ).withTimeout(duration)
    