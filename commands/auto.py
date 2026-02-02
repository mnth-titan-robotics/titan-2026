from typing import TYPE_CHECKING

from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath import units
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians, TrajectoryGenerator, TrajectoryConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from commands.game import Game

import math

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


class Auto:
    def __init__(
            self,
            robot: "RobotContainer",
            game: Game
    ) -> None:
        self._robot = robot
        self._game = game

        self._selectedAuto = cmd.none()

        # Add Robot/Auto to SmartDashboard with options
        self._autos = SendableChooser()
        # Default 'None' auto, does nothing. The second parameter is a function that will be called in our onChange callback below
        self._autos.setDefaultOption("None", cmd.none)
        self._autos.addOption("[Center]", self.auto_center)
        self._autos.addOption("[Test]", self.auto_test)
        # When the selected auto changes call the function (eg: cmd.none(), self.auto_center()) to get the command, then store it in self._auto
        self._autos.onChange(lambda auto: self.set(auto()))
        # Send the list of options to SmartDashboard
        SmartDashboard.putData("Robot/Auto", self._autos)


        

    def get(self) -> Command:
        return self._selectedAuto

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
