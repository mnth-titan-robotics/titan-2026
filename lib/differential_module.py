import math
from typing import Callable, Optional, Tuple
from wpilib import Timer
from wpimath import units
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkFlex
from lib.classes import DifferentialModuleConfig, MotorIdleMode, MotorControllerType
from wpimath.kinematics import MecanumDriveKinematics, MecanumDriveWheelSpeeds
from wpimath.controller import HolonomicDriveController
from commands2 import Command, Subsystem
from wpimath.trajectory import Trajectory

from wpimath.geometry import Pose2d, Rotation2d
class DifferentialModule:
    def __init__(
        self,
        config: DifferentialModuleConfig
    ) -> None:
        self._config = config
        self._baseKey = f'Robot/Drive/Modules/{self._config.location.name}'

        drivingMotorReduction: float = self._config.constants.drivingMotorReduction
        drivingEncoderPositionConversionFactor: float = (
            self._config.constants.wheelDiameter * math.pi) / drivingMotorReduction
        if self._config.constants.drivingMotorControllerType == MotorControllerType.SparkFlex:
            self._drivingMotor = SparkFlex(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
        else:
            self._drivingMotor = SparkMax(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
        self._drivingMotorConfig = SparkBaseConfig()
        (self._drivingMotorConfig
         .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
         .smartCurrentLimit(self._config.constants.drivingMotorCurrentLimit)
         .secondaryCurrentLimit(self._config.constants.drivingMotorCurrentLimit)
         .inverted(self._config.isInverted)
         .voltageCompensation(11.0))
        (self._drivingMotorConfig.encoder
         .positionConversionFactor(drivingEncoderPositionConversionFactor)
         .velocityConversionFactor(drivingEncoderPositionConversionFactor / 60.0)
         )
        self._drivingMotor.configure(
            self._drivingMotorConfig,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        )
        self._drivingEncoder = self._drivingMotor.getEncoder()
        self._drivingEncoder.setPosition(0)

        self._drivingTargetSpeed: units.meters_per_second = 0

    def getMotorController(self) -> SparkMax:
        return self._drivingMotor

    def getPosition(self) -> float:
        return self._drivingEncoder.getPosition()

    def getVelocity(self) -> float:
        return self._drivingEncoder.getVelocity()

    def setVelocity(self, v: float) -> None:
        self._drivingMotor.set(v)

    def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
        idleMode = SparkBaseConfig.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else SparkBaseConfig.IdleMode.kBrake
        self._drivingMotor.configure(
            SparkBaseConfig().setIdleMode(idleMode),
            SparkBase.ResetMode.kNoResetSafeParameters,
            SparkBase.PersistMode.kNoPersistParameters)

    def reset(self) -> None:
        self._drivingEncoder.setPosition(0)

    def _updateTelemetry(self) -> None:
        # SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Target', self._drivingTargetSpeed)
        # SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Actual', self._drivingEncoder.getVelocity())
        # SmartDashboard.putNumber(f'{self._baseKey}/Driving/Position', self._drivingEncoder.getPosition())
        pass


class DifferentialControllerCommand(Command):
    """
    A command that uses two PID controllers (:class:`wpimath.controller.PIDController`)
    and a HolonomicDriveController (:class:`wpimath.controller.HolonomicDriveController`)
    to follow a trajectory (:class:`wpimath.trajectory.Trajectory`) with a swerve drive.

    This command outputs the raw desired Swerve Module States (:class:`wpimath.kinematics.SwerveModuleState`) in an
    array. The desired wheel and module rotation velocities should be taken from those and used in
    velocity PIDs.

    The robot angle controller does not follow the angle given by the trajectory but rather goes
    to the angle given in the final state of the trajectory.
    """

    def __init__(
        self,
        trajectory: Trajectory,
        pose: Callable[[], Pose2d],
        kinematics: MecanumDriveKinematics,
        controller: HolonomicDriveController,
        outputModuleStates: Callable[[MecanumDriveWheelSpeeds], None],
        requirements: Tuple[Subsystem],
        desiredRotation: Optional[Callable[[], Rotation2d]] = None,
    ) -> None:
        """
        Constructs a new SwerveControllerCommand that when executed will follow the
        provided trajectory. This command will not return output voltages but
        rather raw module states from the position controllers which need to be put
        into a velocity PID.

        Note: The controllers will *not* set the outputVolts to zero upon
        completion of the path- this is left to the user, since it is not
        appropriate for paths with nonstationary endstates.

        :param trajectory:         The trajectory to follow.
        :param pose:               A function that supplies the robot pose - use one of the odometry classes to
                                   provide this.
        :param kinematics:         The kinematics for the robot drivetrain. Can be kinematics for 2/3/4/6
                                   SwerveKinematics.
        :param controller:         The HolonomicDriveController for the drivetrain.
                                   If you have x, y, and theta controllers, pass them into
                                   HolonomicPIDController.
        :param outputModuleStates: The raw output module states from the position controllers.
        :param requirements:       The subsystems to require.
        :param desiredRotation:    (optional) The angle that the drivetrain should be
                                   facing. This is sampled at each time step. If not specified, that rotation of
                                   the final pose in the trajectory is used.
        """
        super().__init__()
        self._trajectory = trajectory
        self._pose = pose
        self._kinematics = kinematics
        self._outputModuleStates = outputModuleStates
        self._controller = controller
        if desiredRotation is None:
            self._desiredRotation = trajectory.states()[-1].pose.rotation
        else:
            self._desiredRotation = desiredRotation

        self._timer = Timer()
        self.addRequirements(*requirements)

    def initialize(self):
        self._timer.restart()

    def execute(self):
        curTime = self._timer.get()
        desiredState = self._trajectory.sample(curTime)

        targetChassisSpeeds = self._controller.calculate(
            self._pose(), desiredState, self._desiredRotation()
        )
        wheelSpeeds = self._kinematics.toWheelSpeeds(targetChassisSpeeds)
        
        self._outputModuleStates(wheelSpeeds)

    def end(self, interrupted):
        self._timer.stop()

    def isFinished(self):
        return self._timer.hasElapsed(self._trajectory.totalTime())