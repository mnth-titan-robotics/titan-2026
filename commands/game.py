import math
from typing import TYPE_CHECKING, Callable
from commands2 import Command
from wpimath.controller import HolonomicDriveController, PIDController, ProfiledPIDControllerRadians
from wpimath.geometry import Pose2d
from wpimath.trajectory import Trajectory, TrapezoidProfileRadians
from lib.differential_module import DifferentialControllerCommand
import constants

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


class Game:
    def __init__(self, robot: "RobotContainer"):
        self._robot = robot

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
            get_pose: Callable[[], Pose2d],
            trajectory: Trajectory) -> Command:
        """Returns a command that drives the robot along the provided trajectory"""
        DriveConstants = constants.Subsystems.Drive
        driveSubsystem = self._robot._drive
        
        x_controller = PIDController(
            P=DriveConstants.TranslationPID.P,
            I=DriveConstants.TranslationPID.I,
            D=DriveConstants.TranslationPID.D)
        y_controller = PIDController(
            P=DriveConstants.TranslationPID.P,
            I=DriveConstants.TranslationPID.I,
            D=DriveConstants.TranslationPID.D)
        theta_controller = ProfiledPIDControllerRadians(
            Kp=DriveConstants.RotationPID.P,
            Ki=DriveConstants.RotationPID.I,
            Kd=DriveConstants.RotationPID.D,
            constraints=TrapezoidProfileRadians.Constraints(
                maxVelocity=DriveConstants.MaxVelocity,
                maxAcceleration=DriveConstants.MaxAcceleration
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
            kinematics=DriveConstants.Kinematics,
            controller=drive_controller,
            outputModuleStates=driveSubsystem.setWheelSpeeds,
            requirements=(driveSubsystem)
        )
