from typing import NamedTuple
from lib.enums import *
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform3d


class PID(NamedTuple):
    P: float
    I: float
    D: float


class Tolerance(NamedTuple):
    error: float
    errorDerivative: float


@dataclass(frozen=True, slots=True)
class DriftCorrectionConstants:
    rotationPID: PID
    rotationTolerance: Tolerance


@dataclass(frozen=True, slots=True)
class TargetAlignmentConstants:
    rotationPID: PID
    rotationTolerance: Tolerance
    rotationSpeedMax: units.radians_per_second  # type: ignore
    rotationHeadingModeOffset: units.degrees
    rotationTranslationModeOffset: units.degrees
    translationPID: PID
    translationTolerance: Tolerance
    translationSpeedMax: units.meters_per_second


@dataclass(frozen=True, slots=True)
class SwerveModuleConstants:
    wheelDiameter: units.meters
    wheelBevelGearTeeth: int
    wheelSpurGearTeeth: int
    wheelBevelPinionTeeth: int
    drivingMotorPinionTeeth: int
    drivingMotorFreeSpeed: units.revolutions_per_minute
    drivingMotorControllerType: MotorControllerType
    drivingMotorCurrentLimit: int
    drivingMotorPID: PID
    turningMotorCurrentLimit: int
    turningMotorPID: PID


@dataclass(frozen=True, slots=True)
class SwerveModuleConfig:
    location: ModuleLocation
    drivingMotorCANId: int
    turningMotorCANId: int
    turningOffset: units.radians
    translation: Translation2d
    constants: SwerveModuleConstants


@dataclass(frozen=True, slots=True)
class DifferentialModuleConstants:
    wheelDiameter: units.meters
    drivingMotorControllerType: MotorControllerType
    drivingMotorCurrentLimit: int
    drivingMotorReduction: float


@dataclass(frozen=True, slots=True)
class DifferentialModuleConfig:
    location: ModuleLocation
    drivingMotorCANId: int
    isInverted: bool
    constants: DifferentialModuleConstants


class DifferentialDriveModulePositions(NamedTuple):
    left: float
    right: float


@dataclass(frozen=True, slots=True)
class ObjectSensorConfig:
    cameraName: str
    cameraTransform: Transform3d


@dataclass(frozen=True, slots=True)
class PositionControlModuleConstants:
    motorTravelDistance: units.inches
    motorControllerType: MotorControllerType
    motorCurrentLimit: int
    motorReduction: float
    motorPID: PID
    motorMotionMaxVelocityRate: units.percent
    motorMotionMaxAccelerationRate: units.percent
    allowedClosedLoopError: float
    motorSoftLimitForward: float
    motorSoftLimitReverse: float
    motorResetSpeed: units.percent


@dataclass(frozen=True, slots=True)
class PositionControlModuleConfig:
    moduleBaseKey: str
    motorCANId: int
    leaderMotorCANId: int | None
    constants: PositionControlModuleConstants
