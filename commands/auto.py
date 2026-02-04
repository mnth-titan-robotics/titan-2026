from typing import TYPE_CHECKING

from commands2 import Command, cmd

from wpilib import SendableChooser, SmartDashboard
from wpimath import units
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians, TrajectoryGenerator, TrajectoryConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from commands.game import Game
from pathplannerlib.auto import AutoBuilder, DriveFeedforwards
from pathplannerlib.controller import PPHolonomicDriveController
from wpilib import DriverStation
from pathplannerlib.path import PathPlannerPath
import math
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

        def output(chassisSpeeds: ChassisSpeeds, driveFeedForward: DriveFeedforwards) -> None:
            self._robot._drive._set_chassis_speeds(chassisSpeeds)

        AutoBuilder.configure(
            self._robot._localization.get_pose2d,
            self._robot._localization.reset_pose2d,
            self._robot._drive._get_chassis_speeds,
            output,
            PPHolonomicDriveController(DriveConstants.kPathPlannerTranslationConstants, DriveConstants.kPathPlannerRotationConstants),
            DriveConstants.PathPlannerConfig,
            shouldFlipPath,
            self._robot._drive
        )

        # Add Robot/Auto to SmartDashboard with options
        # Default 'None' auto, does nothing. The second parameter is a function that will be called in our onChange callback below
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Robot/Auto", self.autoChooser)

    def get(self):
        return self.autoChooser.getSelected()

    def set(self, autoCmd: Command) -> None:
        self._selectedAuto = autoCmd

    def auto_center(self) -> Command:
        # Move forward at 25% speed for 3.25s, then stop
        drive = self._robot._drive
        speeds = ChassisSpeeds(vx=0.25)
        return cmd.sequence(
            drive.drive_command(speeds).withTimeout(3.25),
            drive.stopCommand().withTimeout(0.1)
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
