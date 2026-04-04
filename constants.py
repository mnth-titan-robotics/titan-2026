from wpimath import units
import math

from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import FlippingUtil
from wpimath import units
from wpimath.geometry import Translation2d, Pose2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, MecanumDriveKinematics
from wpimath.trajectory import TrapezoidProfile

from lib.classes import MotorControllerType, DifferentialModuleConstants, DifferentialModuleConfig, DriftCorrectionConstants, PID, Tolerance
from lib.enums import ModuleLocation

class FieldConstants:
    kBlueHubPose = Pose2d(Translation2d(4.5, 4.025), Rotation2d())
    kRedHubPose = FlippingUtil.flipFieldPose(kBlueHubPose)
    kLeftPass = Pose2d(Translation2d(2.75, 6.25), Rotation2d())
    kRightPass = Pose2d(Translation2d(2.75, 1.5), Rotation2d())
ENABLE_TELEMETRY = True

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
        kTurnLatency: float = 0.06 # Delay between a commanded turn and when we can expect to reach the setpoint

        TranslationPID = PID(P=1.5, I=0.0, D=0.1)
        RotationPID = PID(P=3.0, I=0.0, D=0.1)

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
        kFrontLeftChassisAngularOffset = math.pi / 2
        kFrontRightChassisAngularOffset = math.pi
        kRearLeftChassisAngularOffset = 0.0
        kRearRightChassisAngularOffset = -math.pi / 2

        # SPARK MAX CAN IDs
        kFrontLeftDrivingCanId: int = 1
        kRearLeftDrivingCanId: int = 7
        kFrontRightDrivingCanId: int = 3
        kRearRightDrivingCanId: int = 5

        kFrontLeftTurningCanId: int = 2
        kRearLeftTurningCanId: int = 8
        kFrontRightTurningCanId: int = 4
        kRearRightTurningCanId: int = 6

        kGyroReversed: bool = False

    class Launcher:
        MotorCurrentLimit = 60
        MotorVComp = 12
        LaunchSpeed = 3.3
        IdleSpeed = 0.2
        LeftMotorId = 20
        RightMotorId = 21
        # Minimum speed in RPM required for the launcher to reliably fire projectiles
        MinLaunchSpeed = 3.0
        DistanceToSpeedLookup = { units.meters(2.75): 3.0, units.meters(5.5): 3.82 }
        LauncherPID = PID(0.7, 0.001, 0.6)
        # 1:1000 1k RPM
        # 1:1.5 gear reduction
        PositionConversionFactor = 1.0 / 1000.0 / 1.5
        VelocityConversionFactor = PositionConversionFactor

    class Intake:
        MotorId = 12
        MotorCurrentLimit = 25
        MotorVComp = 12
        IntakeSpeed = 0.6
        ReverseSpeed = -0.5

    class IntakeExtender:
        LeftMotorId = 10
        RightMotorId = 11
        # 4:1, 3:1, 4:1 MAXPlanetary with 64t/26t pulleys
        GearReduction = 1.0 / 4.0 / 3.0 / 4.0 / (64.0 / 26.0)
        MotorCurrentLimit = 25
        MotorVComp = 12
        MaxAcceleration = 0.5
        AllowedProfileError = 0.05
        MotorSpeed = 0.5
        ReverseSpeed = -0.5

        ForwardLimit = 90.0
        ReverseLimit = -90.0
        RetractPosition = 0.0
        FlapPosition = 0.2
        ExtendPosition = 0.28
        PID = (1.7, 0.0, 0.01)
    
    class Indexer:
        MotorId = 16
        MotorCurrentLimit = 25
        MotorVComp = 12
        IndexerSpeed = 0.9
        ReverseSpeed = -0.5

    class Climber:
        MotorId = 15
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