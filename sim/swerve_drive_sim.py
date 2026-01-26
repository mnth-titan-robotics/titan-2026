import math
import random
import wpimath
from subsystems.max_swerve_module import MAXSwerveModule
from rev import SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim
from wpimath.system.plant import DCMotor
from wpimath.filter import SlewRateLimiter
from wpimath.controller import PIDController
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from subsystems.drive import Drive, DriveConstants
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import RobotController

# Simulation constants
# 5-G acceleration
kdrivingMotorSimSlew: float = 9.8 * 5
# The "free spinning" speed of the turning motor in rad/s
kturningMotorSimSpeed: float = 1.0  # 31.0
kturningMotorSimD: float = 0.0


def clamp(val: float, a: float, b: float) -> float:
    return max(a, min(b, val))


def clampPose(pose: Pose2d) -> Pose2d:
    translation: Translation2d = pose.translation()
    translation = Translation2d(
        clamp(translation.X(), 0, 16.49),
        clamp(translation.Y(), 0, 8.10)
    )
    return Pose2d(translation, pose.rotation())


def _createSim(motor: SparkFlex | SparkMax) -> SparkFlexSim | SparkMaxSim:
    match motor:
        case SparkFlex():
            return SparkFlexSim(motor, DCMotor.NEO(1))
        case SparkMax():
            return SparkMaxSim(motor, DCMotor.NEO(1))


class SwerveDriveSim:
    _pose: Pose2d = Pose2d()
    class SwerveModuleSim:
        def __init__(self, module: MAXSwerveModule):
            self._driveSparkSim = _createSim(module._drivingSpark)
            self._turnSparkSim = _createSim(module._turningSpark)
            self._angularOffset = module._chassisAngularOffset

            # Calculates the velocity of the drive motor, simulating inertia and friction
            self._driveRateLimiter = SlewRateLimiter(kdrivingMotorSimSlew)

            # Calculates the velocity of the turn motor
            self._turnController = PIDController(
                kturningMotorSimSpeed,
                0,
                kturningMotorSimD
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
                self._driveSparkSim.getVelocity(),
                Rotation2d(self._turnSparkSim.getPosition() - self._angularOffset)
            )

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
        for module in self._modules:
            module.simulationPeriodic(vBus, dt)
        moduleStates = tuple(module.getState() for module in self._modules)
        chassisSpeeds: ChassisSpeeds = self._kinematics.toChassisSpeeds(moduleStates)  # type: ignore
        return chassisSpeeds

    def getPose(self) -> Pose2d:
        return self._pose

    def setPose(self, pose: Pose2d) -> None:
        self._pose = pose
