from wpimath import units
import math

from pathplannerlib.config import RobotConfig, PIDConstants
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics, MecanumDriveKinematics
from wpimath.trajectory import TrapezoidProfile

from lib.classes import MotorControllerType, DifferentialModuleConstants, DifferentialModuleConfig, DriftCorrectionConstants, PID, Tolerance
from lib.enums import ModuleLocation


class Controllers:
    DriverPort = 0
    OperatorPort = 1
    kDriveDeadband: float = 0.1


class NeoMotorConstants:
    kFreeSpeedRpm: float = 5676


class Subsystems:
    class Drive:
        PathPlannerConfig = RobotConfig.fromGUISettings()
        kPathPlannerTranslationConstants = PIDConstants(4.0, 0.0, 0.1)
        kPathPlannerRotationConstants = PIDConstants(3.0, 0.0, 0.1)

        class Mecanum:
            kDriftCorrectionConstants = DriftCorrectionConstants(
                rotationPID=PID(0.01, 0, 0),
                rotationTolerance=Tolerance(0.5, 1.0)
            )

            kMaxSpeedMetersPerSecond: units.meters_per_second = units.meters_per_second(12.0)
            kMaxAcceleration: units.meters_per_second_squared = units.meters_per_second_squared(3.0)
            kMaxAngularSpeed: float = 2.0 * math.pi
            kTrackWidth: units.meters = units.inchesToMeters(23)
            kWheelBase: units.meters = units.inchesToMeters(27.5)
                
            TranslationPID = PID(P=1.5, I=0.0, D=0.1)
            RotationPID = PID(P=1.0, I=0.0, D=0.1)

            kTranslationSpeedMax: units.meters_per_second = 4.8
            kRotationSpeedMax: units.radians_per_second = 2 * math.pi  # type: ignore

            kInputLimitDemo: units.percent = 0.5
            kInputRateLimitDemo: units.percent = 0.33

            _differentialModuleConstants = DifferentialModuleConstants(
                wheelDiameter=units.inchesToMeters(3.0),
                drivingMotorControllerType=MotorControllerType.SparkMax,
                drivingMotorCurrentLimit=50,
                drivingMotorReduction=8.46
            )

            kDifferentialModuleConfigs: tuple[DifferentialModuleConfig, ...] = (
                DifferentialModuleConfig(
                    location=ModuleLocation.LeftFront,
                    drivingMotorCANId=5,
                    isInverted=False,
                    constants=_differentialModuleConstants),
                DifferentialModuleConfig(
                    location=ModuleLocation.LeftRear,
                    drivingMotorCANId=3,
                    isInverted=False,
                    constants=_differentialModuleConstants),
                DifferentialModuleConfig(
                    location=ModuleLocation.RightFront,
                    drivingMotorCANId=7,
                    isInverted=True,
                    constants=_differentialModuleConstants),
                DifferentialModuleConfig(
                    location=ModuleLocation.RightRear,
                    drivingMotorCANId=1,
                    isInverted=True,
                    constants=_differentialModuleConstants)
            )

            kDriveKinematics = MecanumDriveKinematics(
                Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0)
            )

        kMaxSpeedMetersPerSecond: float = 12
        kMaxAngularSpeed: float = 2.0 * math.pi
        kMaxAcceleration: float = 1.0  # meters per second squared

        TranslationPID = PID(P=1.5, I=0.0, D=0.1)
        RotationPID = PID(P=1.0, I=0.0, D=0.1)

        # Chassis configuration
        # Distance between centers of right and left wheels on robot
        kTrackWidth: units.meters = units.inchesToMeters(26.5)
        
        # Distance between front and back wheels on robot
        kWheelBase: units.meters = units.inchesToMeters(26.5)

        kDriveKinematics: SwerveDrive4Kinematics = SwerveDrive4Kinematics(
            Translation2d(kWheelBase / 2, kTrackWidth / 2),
            Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        )

         # Angular offsets of the modules relative to the chassis in radians
        kFrontLeftChassisAngularOffset = -math.pi / 2
        kFrontRightChassisAngularOffset = 0
        kRearLeftChassisAngularOffset = math.pi
        kRearRightChassisAngularOffset = math.pi / 2

        # SPARK MAX CAN IDs
        kFrontLeftDrivingCanId: int = 5
        kRearLeftDrivingCanId: int = 3
        kFrontRightDrivingCanId: int = 7
        kRearRightDrivingCanId: int = 1

        kFrontLeftTurningCanId: int = 6
        kRearLeftTurningCanId: int = 4
        kFrontRightTurningCanId: int = 8
        kRearRightTurningCanId: int = 2

        kGyroReversed: bool = False

    class Launcher:
        MotorId = 6
        MotorCurrentLimit = 40
        MotorVComp = 12
        LaunchSpeed = -1.0
        IdleSpeed = 0.2

    class Intake:
        MotorId = 2
        MotorCurrentLimit = 25
        MotorVComp = 10
        IntakeSpeed = 1.0
        ReverseSpeed = -0.5

    class Climber:
        MotorId = 20
        MotorCurrentLimit = 30
        MotorVComp = 12
        ClimbSpeed = 1.0
        LowerSpeed = -0.5


class ModuleConstants:
    # The MAXSwerve module can be configured with one of three pinion gears: 12T,
    # 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    # more teeth will result in a robot that drives faster).
    kDrivingMotorPinionTeeth: int = 14

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps: float = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters: float = 0.0762
    kWheelCircumferenceMeters: float = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    # teeth on the bevel pinion
    kDrivingMotorReduction: float = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps: float = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kTurningMotorReduction: float = 9424.0 / 203.0

    # Simulation constants
    # 5-G acceleration
    kdrivingMotorSimSlew: float = 9.8 * 5
    # Treat the turning motor as free-spinning
    kturningMotorSimSpeed: float = 31.0
    kturningMotorSimD: float = 0.0


class AutoConstants:
    kMaxSpeedMetersPerSecond: float = 3
    kMaxAccelerationMetersPerSecondSquared: float = 3
    kMaxAngularSpeedRadiansPerSecond: float = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared: float = math.pi
    kPXController: float = 1
    kPYController: float = 1
    kPThetaController: float = 1

    # Constraint for the motion profiled robot angle controller
    kThetaControllerConstraints = TrapezoidProfile.Constraints(
    kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared)


class Roller:
    MotorId = 9
    MotorCurrentLimit = 20
    MotorVComp = 10
    EjectSpeed = 1.0
    ReverseSpeed = -1.0