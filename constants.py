from lib.enums import *
from lib.classes import *
from wpimath import units
from wpimath.kinematics import MecanumDriveKinematics
import math


class Controllers:
    DriverPort = 0
    OperatorPort = 1


class Subsystems:
    class Drive:
        TrackWidth: units.meters = units.inchesToMeters(23)
        WheelBase: units.meters = units.inchesToMeters(27.5)

        TranslationSpeedMax: units.meters_per_second = 4.8
        RotationSpeedMax: units.radians_per_second = 2 * math.pi  # type: ignore

        ModuleConstants = DifferentialModuleConstants(
            wheelDiameter=units.inchesToMeters(3.0),
            drivingMotorControllerType=MotorControllerType.SparkMax,
            drivingMotorCurrentLimit=50,
            drivingMotorReduction=8.46
        )

        # Each wheel is a "module", with CAN ids
        ModuleConfigs = (
            DifferentialModuleConfig(
                location=ModuleLocation.LeftFront,
                drivingMotorCANId=10,
                isInverted=False,
                constants=ModuleConstants),
            DifferentialModuleConfig(
                location=ModuleLocation.LeftRear,
                drivingMotorCANId=11,
                isInverted=False,
                constants=ModuleConstants),
            DifferentialModuleConfig(
                location=ModuleLocation.RightFront,
                drivingMotorCANId=12,
                isInverted=True,
                constants=ModuleConstants),
            DifferentialModuleConfig(
                location=ModuleLocation.RightRear,
                drivingMotorCANId=13,
                isInverted=True,
                constants=ModuleConstants)
        )

        # Kinematics provide a model of how the robot will move based on wheel motion.
        # DifferentialDriveKinematics - aka: "Tank Drive"
        # MecanumDriveKinematics - Wheel can only rotate axially, diagonal rollers allow moving side-to-side
        # SwerveDrive4Kinematics - Wheels can rotate (like shopping cart wheels), move forward and backward.
        Kinematics = MecanumDriveKinematics(
            Translation2d(WheelBase / 2.0, TrackWidth / 2.0),
            Translation2d(WheelBase / 2.0, -TrackWidth / 2.0),
            Translation2d(-WheelBase / 2.0, TrackWidth / 2.0),
            Translation2d(-WheelBase / 2.0, -TrackWidth / 2.0)
        )

        # TODO: Tune these PID values
        TranslationPID = PID(1.0, 0.0, 0.0)
        RotationPID = PID(1.0, 0.0, 0.0)

        MaxVelocity = 15
        MaxAcceleration = 3

    class Roller:
        MotorId = 15
        MotorCurrentLimit = 20
        MotorVComp = 10
        EjectSpeed = 1.0
        ReverseSpeed = -1.0
