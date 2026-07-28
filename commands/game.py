from typing import TYPE_CHECKING

from commands2 import Command, cmd
from wpimath.kinematics import ChassisSpeeds
from wpimath import units

import constants
from lib import utils

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


class Game:
    def __init__(self, robot: "RobotContainer"):
        self._robot = robot
        self._localization = robot.localization

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
        ).withName('shakeIntakeCommand')

    def agitateRobotCommand(self) -> Command:
        SHAKE_SPEED = 0.5
        SHAKE_TIME = units.seconds(0.1)
        PAUSE_TIME = units.seconds(0.15)
        forward = ChassisSpeeds(SHAKE_SPEED * constants.Subsystems.Drive.kMaxSpeedMetersPerSecond)
        reverse = ChassisSpeeds(-SHAKE_SPEED * constants.Subsystems.Drive.kMaxSpeedMetersPerSecond)
        return cmd.repeatingSequence(
            self._robot.drive.drive_command(lambda: forward).withTimeout(SHAKE_TIME),
            cmd.waitSeconds(PAUSE_TIME),
            self._robot.drive.drive_command(lambda: reverse).withTimeout(SHAKE_TIME),
            cmd.waitSeconds(PAUSE_TIME)
        ).withName('agitateRobotCommand')

    def pulseIndexerCommand(self) -> Command:
        """Runs the indexer, attempting to feed only one fuel at a time."""
        FEED_TIME = units.seconds(0.2)
        STOP_TIME = units.seconds(0.04)

        return cmd.repeatingSequence(
            self._robot.indexer.feed().withTimeout(FEED_TIME),
            self._robot.indexer.stop().withTimeout(STOP_TIME)
        ).withName('pulseIndexerCommand')

    def feedFuelCommand(self) -> Command:
        """Runs the indexer while 'flapping' the intake"""
        return cmd.parallel(
            self.shakeIntakeCommand(),
            self.pulseIndexerCommand()
        ).withName('feedFuelCommand')

    def feed_and_shoot(self):
        """Runs the launcher, then starts feeding fuel once the launcher is up to speed"""
        return cmd.parallel(
            self._robot.launcher.start(),
            cmd.none()
            .until(self._robot.launcher.is_ready_to_shoot)
            .andThen(cmd.parallel(
                self.feedFuelCommand(),
                self._robot.intake.intake_half_speed()
            )
            )
        ).withName('feed_and_shoot')

    def reverse_intake_and_indexer(self):
        """Runs both the intake and indexer in reverse to dump fuel or to unjam intake or indexer"""
        return cmd.parallel(
            self._robot.intake.reverse(),
            self._robot.indexer.reverse()
        ).withName("reverse_intake_and_indexer")

    def toggleLockOnHub(self) -> Command:
        return self._robot.drive.toggle_lock_command(
            utils.getValueForAlliance(constants.FieldConstants.kBlueHubPose, constants.FieldConstants.kRedHubPose))

    def is_shooting(self) -> bool:
        """Returns True if the launcher is running and the indexer is feeding fuel."""
        return self._robot.launcher.is_ready_to_shoot() and self._robot.indexer.is_feeding()
