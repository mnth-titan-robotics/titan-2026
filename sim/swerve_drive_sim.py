import math
import random
import wpimath
from rev import SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim
from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
from wpimath.system.plant import DCMotor, LinearSystemId
from subsystems.drive import Drive, DriveConstants
from subsystems.max_swerve_module import MAXSwerveModule
from constants import ModuleConstants

# Simulation constants
# 5-G acceleration
kdrivingMotorSimSlew: float = 9.8 * 5
# The "free spinning" speed of the turning motor in rad/s
kturningMotorSimSpeed: float = units.rotationsToRadians(4)
kturningMotorSimD: float = 0.0

kDrivingMotor = DCMotor.NEO(1)
kTurningMotor = DCMotor.NEO550(1)


def clamp(val: float, a: float, b: float) -> float:
    return max(a, min(b, val))


def clampPose(pose: Pose2d) -> Pose2d:
    translation: Translation2d = pose.translation()
    translation = Translation2d(
        clamp(translation.X(), 0, 16.49),
        clamp(translation.Y(), 0, 8.10)
    )
    return Pose2d(translation, pose.rotation())


def _createSim(motor_controller: SparkFlex | SparkMax, motor: DCMotor) -> SparkFlexSim | SparkMaxSim:
    match motor_controller:
        case SparkFlex():
            return SparkFlexSim(motor_controller, motor)
        case SparkMax():
            return SparkMaxSim(motor_controller, motor)


class SwerveDriveSim:
    _pose: Pose2d = Pose2d()

    class SwerveModuleSim:
        def __init__(self, module: MAXSwerveModule):
            self._driveSparkSim = _createSim(module._drivingSpark, kDrivingMotor)
            self._turnSparkSim = _createSim(module._turningSpark, kTurningMotor)
            self._driveMotorSim = DCMotorSim(
                LinearSystemId.DCMotorSystem(
                    kDrivingMotor, 
                    units.kilogram_square_meters(2.0), 
                    ModuleConstants.kDrivingMotorReduction),
                kDrivingMotor
            )
            self._turnMotorSim = DCMotorSim(
                LinearSystemId.DCMotorSystem(
                    kTurningMotor,
                    units.kilogram_square_meters(0.1),
                    ModuleConstants.kTurningMotorReduction),
                kTurningMotor
            )
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
            d_factor = self._driveSparkSim.getRelativeEncoderSim().getVelocityConversionFactor()
            d_v = self._driveSparkSim.getAppliedOutput() * vbus
            self._driveMotorSim.setInputVoltage(d_v)
            self._driveMotorSim.update(dt)
            driveVelocity = d_factor * self._driveMotorSim.getAngularVelocity()
            self._driveSparkSim.iterate(
                driveVelocity,
                vbus,
                dt
            )

            # rad
            t_factor = self._turnSparkSim.getAbsoluteEncoderSim().getVelocityConversionFactor()
            t_v = self._turnSparkSim.getAppliedOutput() * vbus
            self._turnMotorSim.setInputVoltage(t_v)
            self._turnMotorSim.update(dt)
            turnVelocity = t_factor * self._turnMotorSim.getAngularVelocity()
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
