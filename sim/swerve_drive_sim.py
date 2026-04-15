import math
import random
import wpimath
from rev import SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim
from wpilib import RobotController
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.system.plant import DCMotor
from wpimath.system import plant
from subsystems import Drive, DriveConstants, MAXSwerveModule
from constants import ModuleConstants

# Simulation constants
# 5-G acceleration
kdrivingMotorSimSlew: float = 9.8 * 5
# The "free spinning" speed of the turning motor in rad/s
kturningMotorSimSpeed: float = units.rotationsToRadians(4)
kturningMotorSimD: float = 0.0

kDrivingMotor = plant.DCMotor.NEO(1)
kTurningMotor = plant.DCMotor.NEO550(1)
# Estimated MOI, taken from MechanicalAdvantage's Swerve Module Sim example
kDriveMomentOfInertia = units.kilogram_square_meters(0.025)
# Estimated MOI, taken from MechanicalAdvantage's Swerve Module Sim example
kTurningMomentOfInertia = units.kilogram_square_meters(0.004096955)


def clamp(val: float, a: float, b: float) -> float:
    return max(a, min(b, val))


def clampPose(pose: Pose2d) -> Pose2d:
    translation: Translation2d = pose.translation()
    translation = Translation2d(
        clamp(translation.X(), 0, 16.49),
        clamp(translation.Y(), 0, 8.10)
    )
    return Pose2d(translation, pose.rotation())


def _createSim(motor_controller: SparkFlex | SparkMax, motor: plant.DCMotor) -> SparkFlexSim | SparkMaxSim:
    match motor_controller:
        case SparkFlex():
            return SparkFlexSim(motor_controller, motor)
        case SparkMax():
            return SparkMaxSim(motor_controller, motor)


class SwerveDriveSim:
    class SwerveModuleSim:
        def __init__(self, module: MAXSwerveModule):
            self._driveSparkSim = SparkFlexSim(module._drivingSpark, DCMotor.NEO(1))
            self._turnSparkSim = SparkMaxSim(module._turningSpark, DCMotor.NEO(1))
            self._angularOffset = module._chassisAngularOffset

            # Calculates the velocity of the drive motor, simulating inertia and friction
            self._driveRateLimiter = SlewRateLimiter(ModuleConstants.kdrivingMotorSimSlew)

            # Calculates the velocity of the turn motor
            self._turnController = PIDController(
                ModuleConstants.kturningMotorSimSpeed,
                0,
                ModuleConstants.kturningMotorSimD
            )
            self._turnController.enableContinuousInput(-math.pi, math.pi)

            # Randomize starting rotation
            self._turnSparkSim.setPosition(random.uniform(-math.pi, math.pi))

        def simulationPeriodic(self, vbus: float, dt: float):
            # m/s
            targetVelocity = self._driveSparkSim.getSetpoint()
            driveVelocity = self._driveRateLimiter.calculate(targetVelocity)
            self._driveSparkSim.iterate(
                driveVelocity,
                vbus,
                dt
            )

            # rad
            targetAngle = self._turnSparkSim.getSetpoint()
            curAngle = wpimath.angleModulus(self._turnSparkSim.getPosition())
            self._turnController.setSetpoint(targetAngle)
            turnVelocity = self._turnController.calculate(curAngle)
            self._turnSparkSim.iterate(
                turnVelocity,
                vbus,
                dt
            )

        def getState(self) -> SwerveModuleState:
            return SwerveModuleState(
                units.meters_per_second(self._driveSparkSim.getVelocity()),
                Rotation2d(self._turnSparkSim.getPosition() - self._angularOffset)
            )
    _pose: Pose2d = Pose2d()

    def __init__(self, driveSubsystem: Drive):
        self._kinematics = DriveConstants.kDriveKinematics
        self._modules = (
            SwerveDriveSim.SwerveModuleSim(driveSubsystem._frontLeft),
            SwerveDriveSim.SwerveModuleSim(driveSubsystem._frontRight),
            SwerveDriveSim.SwerveModuleSim(driveSubsystem._rearLeft),
            SwerveDriveSim.SwerveModuleSim(driveSubsystem._rearRight)
        )

    def simulationPeriodic(self, dt: float) -> ChassisSpeeds:
        vBus = RobotController.getBatteryVoltage()
        moduleStates = list()
        for module in self._modules:
            module.simulationPeriodic(vBus, dt)
            moduleStates.append(module.getState())
        moduleStates = tuple(moduleStates)
        chassisSpeeds: ChassisSpeeds = self._kinematics.toChassisSpeeds(moduleStates)  # type: ignore
        return chassisSpeeds

    def getPose(self) -> Pose2d:
        return self._pose

    def setPose(self, pose: Pose2d) -> None:
        self._pose = pose
