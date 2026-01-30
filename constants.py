from lib.enums import *
from lib.classes import *
from wpimath import units
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
import math


class Controllers:
    DriverPort = 0
    OperatorPort = 1
    kDriveDeadband: float = 0.1


class NeoMotorConstants:
    kFreeSpeedRpm: float = 5676


class Subsystems:
    class Drive:
        kMaxSpeedMetersPerSecond: float = 12
        kMaxAngularSpeed: float = 2.0 * math.pi

        kTrackWidth: units.meters = units.inchesToMeters(26.5)
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
        kFrontLeftDrivingCanId: int = 11
        kRearLeftDrivingCanId: int = 15
        kFrontRightDrivingCanId: int = 13
        kRearRightDrivingCanId: int = 17

        kFrontLeftTurningCanId: int = 12
        kRearLeftTurningCanId: int = 16
        kFrontRightTurningCanId: int = 14
        kRearRightTurningCanId: int = 18

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