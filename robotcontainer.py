#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum
import commands2
import constants
from subsystems.drive import Drive
from subsystems.launcher import Launcher
from subsystems.intake import Intake
from commands.auto import Auto
from commands.game import Game
from wpimath.kinematics import ChassisSpeeds
from services import Tracker, TrackerConstants, Localization
from services.questnav import QuestNav
from lib import utils

# Create an alias to simplify usage
cmd = commands2.cmd


class RobotContainer:
    """This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    # The enum used as keys for selecting the command to run.
    class CommandSelector(enum.Enum):
        ONE = enum.auto()
        TWO = enum.auto()
        THREE = enum.auto()

    # An example selector method for the selectcommand.
    def select(self) -> CommandSelector:
        """Returns the selector that will select which command to run.
        Can base this choice on logical conditions evaluated at runtime.
        """
        return self.CommandSelector.ONE

    def __init__(self) -> None:
        self._initServices()
        self._initSubsystems()
        self._initControllers()
        self._initCommands()
        self._initControllerBindings()
        
    def _initServices(self):
        self._localization = Localization()
        self._tracker = Tracker(TrackerConstants(), self._localization.get_pose, self._localization.get_velocity)

    def _initSubsystems(self):
        """Initializes subsystems. Should only be called from __init__"""
        self._drive = Drive(self._tracker)
        self._launcher = Launcher()
        self._intake = Intake()

    def _initControllers(self):
        self._driverController = commands2.button.CommandXboxController(
            constants.Controllers.DriverPort
        )
        self._operatorController = commands2.button.CommandXboxController(
            constants.Controllers.OperatorPort
        )

    def _initCommands(self):
        """Initializes commands. Should only be called from __init__"""
        self.game = Game(self)
        self.auto = Auto(self)

        # Set default commands for all subsystems
        self._drive.setDefaultCommand(
            self._drive.driveCommand(
                lambda: ChassisSpeeds(
                    -self._driverController.getLeftY(),
                    -self._driverController.getLeftX(),
                    -self._driverController.getRightY())
            )
        )
        self._launcher.setDefaultCommand(
            self._launcher.stop()
        )
        self._intake.setDefaultCommand(
            self._intake.stop()
        )

    def _initControllerBindings(self) -> None:
        """Use this method to define your button->command mappings."""
        # All possible XBOX controller inputs are included below for easy reference.
        # Leaving unused, commented lines helps us keep track of what inputs aren't being used.
        # self.driver.rightStick().whileTrue(cmd.none())
        # self.driver.leftStick().whileTrue(cmd.none())
        self._driverController.leftTrigger().whileTrue(self._intake.intake())
        self._driverController.rightTrigger().whileTrue(self._launcher.start())
        # self.driver.rightBumper().whileTrue(cmd.none())
        # self.driver.leftBumper().whileTrue(cmd.none())
        # self.driver.povUp().whileTrue(cmd.none())
        # self.driver.povDown().whileTrue(cmd.none())
        # self.driver.povLeft().whileTrue(cmd.none())
        # self.driver.povRight().whileTrue(cmd.none())
        # self.driver.a().whileTrue(cmd.none())
        # self.driver.b().whileTrue(cmd.none())
        # self.driver.y().whileTrue(cmd.none())
        # self.driver.x().whileTrue(cmd.none())
        # self.driver.start().whileTrue(cmd.none())
        # self.driver.back().whileTrue(cmd.none())
        # self._operatorController.rightTrigger().whileTrue(cmd.none())

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {Robot} class.

        :returns: the command to run in autonomous
        """
        return self.auto.get()
