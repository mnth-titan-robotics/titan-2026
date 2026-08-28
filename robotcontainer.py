#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import enum
import commands2
import constants
from subsystems import CANHealthMonitor, Drive, Launcher, Intake, IntakeExtender, Indexer, LEDSubsystem, LightPattern
from commands.auto import Auto
from commands.game import Game
from services import Tracker, TrackerConstants, Localization
from lib import utils, logger
from lib.gyro_pigeon import Gyro_Pigeon2
from wpilib import DriverStation, RobotBase

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
        self.invertLeft = -1.0
        self.invertRight = -1.0
        self._initServices()
        self._initSubsystems()
        self._initControllers()
        self._initCommands()
        self._initControllerBindings()
        logger.start()

    def updateLights(self, robot: RobotBase):
        """Update the lights based on the current robot state"""
        if self.can_monitor.is_can_faulted:
            self.lights.set_pattern(LightPattern.ERROR)
        elif not DriverStation.isDSAttached():
            self.lights.set_pattern(LightPattern.DS_DISCONNECTED)
        elif robot.isDisabled():
            self.lights.set_pattern(LightPattern.SHOWTIME)
        elif self.game.is_shooting():
            self.lights.set_pattern(LightPattern.SHOOTING)
        elif self.launcher.is_ready_to_shoot():
            self.lights.set_pattern(LightPattern.READY_TO_SHOOT)
        else:
            self.lights.set_pattern(LightPattern.IDLE)

    def _initServices(self):
        # The gyro is created here rather than inside Drive so that Localization can share
        # the same instance - it needs a heading source to fall back on when QuestNav drops,
        # and a second Pigeon2 object would not see Drive's zeroHeading() offset.
        self.gyro = Gyro_Pigeon2()
        self.localization = Localization(self.gyro)
        self.tracker = Tracker(TrackerConstants(), self.localization.get_pose2d, self.localization.get_velocity)

    def _initSubsystems(self):
        """Initializes subsystems. Should only be called from __init__"""
        self.drive = Drive(self.localization, self.tracker, self.gyro)
        self.launcher = Launcher(self.tracker)
        self.indexer = Indexer()
        self.intake = Intake()
        self.intakeExtender = IntakeExtender()
        self.lights = LEDSubsystem()
        self.can_monitor = CANHealthMonitor()

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
        self.auto = Auto(self, self.game)

        # Set default commands for all subsystems
        self.drive.setDefaultCommand(
            self.drive.drive_joystick_command(
                lambda: self.invertLeft * utils.apply_deadband(self._driverController.getLeftY()),
                lambda: self.invertLeft *utils.apply_deadband(self._driverController.getLeftX()),
                lambda: self.invertRight * utils.apply_deadband(self._driverController.getRightX()) ** 3
            )
        )
        self.launcher.setDefaultCommand(
            self.launcher.stop()
        )
        self.indexer.setDefaultCommand(
            self.indexer.stop()
        )
        self.intake.setDefaultCommand(
            self.intake.stop()
        )
        self.intakeExtender.setDefaultCommand(
            self.intakeExtender.stop()
        )

    def _initControllerBindings(self) -> None:
        """Use this method to define your button->command mappings."""
        # All possible XBOX controller inputs are included below for easy reference.
        # Leaving unused, commented lines helps us keep track of what inputs aren't being used.
        # ==========================================
        # DRIVER
        # ==========================================
        def invert_left():
            self.invertLeft *= -1.0
        # self._driverController.rightStick().whileTrue(cmd.runOnce(invert_right))
        self._driverController.leftStick().whileTrue(cmd.runOnce(invert_left))
        # self._driverController.leftTrigger().whileTrue(cmd.none())
        # self._driverController.rightTrigger().whileTrue(cmd.none())
        # self._driverController.rightBumper().whileTrue(cmd.none())
        # self._driverController.leftBumper().whileTrue(cmd.none())
        # self._driverController.povUp().whileTrue(cmd.none())
        # self._driverController.povDown().whileTrue(cmd.none())
        # self._driverController.povLeft().whileTrue(cmd.none())
        # self._driverController.povRight().whileTrue(cmd.none())
        self._driverController.a().whileTrue(self.game.agitateRobotCommand())
        # self._driverController.b().whileTrue(cmd.none())
        #   UNTESTED PLEASE DON'T TRY TS WITHOUT TESTING
        self._driverController.y().onTrue(self.drive.toggle_field_relative_command())
        self._driverController.x().debounce(0.1).onTrue(self.game.toggleLockOnHub())
        #self._driverController.start().onTrue(self.drive.toggle_field_relative_command())
        self._driverController.back().and_(self._driverController.start()).onTrue(self.game.driverResetCommand())

        # ==========================================
        # OPERATOR
        # ==========================================
        # self._operatorController.rightStick().whileTrue(cmd.none())
        # self._operatorController.leftStick().whileTrue(cmd.none())
        self._operatorController.leftTrigger().whileTrue(self.intake.intake())
        self._operatorController.rightTrigger().whileTrue(self.launcher.start())
        self._operatorController.rightBumper().whileTrue(self.game.feed_and_shoot())
        self._operatorController.leftBumper().whileTrue(self.intake.reverse())
        self._operatorController.povDown().whileTrue(self.intakeExtender.extend())
        self._operatorController.povUp().whileTrue(self.intakeExtender.retract())
        self._operatorController.povLeft().whileTrue(self.launcher.decrease_bias())
        self._operatorController.povRight().whileTrue(self.launcher.increase_bias())
        self._operatorController.a().whileTrue(self.game.pulseIndexerCommand())
        self._operatorController.b().whileTrue(self.indexer.reverse())
        self._operatorController.y().debounce(0.25).onTrue(self.launcher.toggle_auto_speed())
        # self._operatorController.x().whileTrue(cmd.none())
        #self._operatorController.start().whileTrue(self.game.feed_and_shoot())
        self._operatorController.back().whileTrue(self.game.operatorResetCommand())

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {Robot} class.

        :returns: the command to run in autonomous
        """
        return self.auto.get()