import math
import random
import wpimath
from rev import SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim
from wpilib import RobotController
from wpilib.simulation import DCMotorSim, FlywheelSim
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState
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
    _pose: Pose2d = Pose2d()

    class SwerveModuleSim:
        def __init__(self, module: MAXSwerveModule):
            turnPlant = plant.LinearSystemId.flywheelSystem(
                kTurningMotor, ModuleConstants.kTurningMotorReduction, kTurningMomentOfInertia
            )
            drivePlant = plant.LinearSystemId.flywheelSystem(
                kDrivingMotor, ModuleConstants.kDrivingMotorReduction, kDriveMomentOfInertia
            )
            self._turnFlywheel = FlywheelSim(turnPlant, kTurningMotor)
            self._driveFlywheel = FlywheelSim(drivePlant, kDrivingMotor)
            self._driveSparkSim = _createSim(module._drivingSpark, kDrivingMotor)
            self._turnSparkSim = _createSim(module._turningSpark, kTurningMotor)
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
            voltage = vbus * self._turnSparkSim.getAppliedOutput()
            self._turnFlywheel.setInput([voltage])
            self._turnFlywheel.update(dt)
            velocity = self._turnFlywheel.getAngularVelocity()
            velocity_rpm = units.radiansPerSecondToRotationsPerMinute(velocity)
            velocity_final = velocity_rpm #/ self._turnSparkSim.getRelativeEncoderSim().getVelocityConversionFactor()
            self._turnSparkSim.iterate(
                velocity_final,
                vbus,
                dt
            )

            voltage = vbus * self._driveSparkSim.getAppliedOutput()
            self._driveFlywheel.setInput([voltage])
            self._driveFlywheel.update(dt)
            velocity = self._driveFlywheel.getAngularVelocity()
            velocity_rpm = units.radiansPerSecondToRotationsPerMinute(velocity)
            velocity_final = velocity_rpm / self._driveSparkSim.getRelativeEncoderSim().getVelocityConversionFactor()
            self._driveSparkSim.iterate(
                velocity_final,
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
