#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing

from robotcontainer import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """Add any setup/cleanup that needs to be done when starting teleop, auto or disabling the robot here"""
    container: typing.Optional[RobotContainer] = None
    autonomousCommand: typing.Optional[commands2.Command] = None

    def __init__(self):
        """Calls the TimedCommandRobot __init__ method"""
        super().__init__()

    def robotInit(self) -> None:
        """
        Called once when the robot is first started up and should be used for any
        initialization code.
        """
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    def robotPeriodic(self):
        """Called periodically for all modes"""

    def disabledInit(self) -> None:
        """Called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """Called periodically when disabled"""

    def autonomousInit(self) -> None:
        """Called once when beginning autonomous. Should schedule the selected autonomous command."""
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """Called periodically during autonomous"""

    def teleopInit(self) -> None:
        """Called once each time the robot enters TeleOp mode."""
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self) -> None:
        """Called periodically during operator control"""

    def testInit(self) -> None:
        """Called once for each test."""
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """Called periodically when running tests"""

    def _simulationInit(self):
        """Called once when initializing simulation, after robotInit"""

    def _simulationPeriodic(self):
        """Called periodically during simulation"""


if __name__ == "__main__":
    wpilib.run(Robot)
