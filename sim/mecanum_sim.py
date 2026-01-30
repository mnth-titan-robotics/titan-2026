import math
from rev import SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim, SparkSim
from wpilib import RobotController
from wpilib.simulation import DCMotorSim
from wpimath import units
from wpimath.kinematics import ChassisSpeeds
from wpimath.system.plant import DCMotor, LinearSystemId

from lib.differential_module import DifferentialModule
from subsystems.mechdrive import Drive, DriveConstants

kDrivingMotor = DCMotor.NEO(1)


def _createSim(motor_controller: SparkFlex | SparkMax, motor: DCMotor) -> SparkSim | SparkMaxSim:
    match motor_controller:
        case SparkFlex():
            return SparkFlexSim(motor_controller, motor)
        case SparkMax():
            return SparkMaxSim(motor_controller, motor)


class DifferentialModuleSim:
    def __init__(self, module: DifferentialModule):
        self._sparkSim = _createSim(module.getMotorController(), kDrivingMotor)
        self._motorSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(
                kDrivingMotor,
                units.kilogram_square_meters(0.02),
                DriveConstants._differentialModuleConstants.drivingMotorReduction),
            kDrivingMotor
        )
        # Velocity conversion factor - motor rpm -> wheel mps
        # motorsim.getAngularVelocity() returns in radians per second
        # Need to convert from rad/s to m/s
        # 2pi/1 (rad/s -> rot/s) * 60/1 (rot/s -> rpm)
        self._factor = 60.0 / math.pi / 2.0 * self._sparkSim.getRelativeEncoderSim().getVelocityConversionFactor()

    def simulationPeriodic(self, vbus: float, dt: float):
        voltage = self._sparkSim.getAppliedOutput() * vbus
        self._motorSim.setInputVoltage(voltage)
        self._motorSim.update(dt)
        velocity = self._factor * self._motorSim.getAngularVelocity() * 10.0
        self._sparkSim.iterate(
            velocity,
            vbus,
            dt
        )


class MecanumSim:
    def __init__(self, driveSubsystem: Drive):
        self._subsystem = driveSubsystem
        self._kinematics = DriveConstants.kDriveKinematics
        self._motor_sims = tuple([DifferentialModuleSim(x) for x in driveSubsystem._differentialModules.values()])

    def simulationPeriodic(self, dt: float) -> ChassisSpeeds:
        vBus = RobotController.getBatteryVoltage()
        for motor_sim in self._motor_sims:
            motor_sim.simulationPeriodic(vBus, dt)

        return self._kinematics.toChassisSpeeds(self._subsystem._get_wheel_speeds())
