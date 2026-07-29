from commands2 import Command, cmd
from phoenix6 import BaseStatusSignal
from wpilib import DriverStation, RobotBase, SmartDashboard, Timer
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d
from phoenix6.hardware import Pigeon2
from lib import logger, utils
import constants


Constants = constants.Pigeon2


class Gyro_Pigeon2():
    def __init__(self) -> None:
        self._baseKey = f'Robot/Sensors/Pigeon2_Gyro'

        self._gyro = Pigeon2(
            device_id=Constants.kDeviceID,
            canbus=Constants.kCANBus
        )

        self._angleAdjustment: units.degrees = 0

        # Cache the StatusSignal objects. Calling get_yaw() etc. performs a refresh on every
        # call, and getHeading() is hit several times per loop (odometry, telemetry, the drive
        # command). Holding the signals lets us refresh all of them exactly once per loop.
        self._yawSignal = self._gyro.get_yaw()
        # NOTE: yaw rate is angular velocity about Z. This previously read the X axis, so
        # getYawRate() was returning the roll rate.
        self._yawRateSignal = self._gyro.get_angular_velocity_z_world()
        self._pitchSignal = self._gyro.get_pitch()
        self._rollSignal = self._gyro.get_roll()

        self._rawYaw: units.degrees = 0.0
        self._yawRate: units.degrees_per_second = 0.0
        self._pitch: units.degrees = 0.0
        self._roll: units.degrees = 0.0
        self._connected: bool = False
        self._lastRefresh: units.seconds = -1.0

        self._refresh()
        utils.addRobotPeriodic(self._periodic)

    def _refresh(self) -> None:
        """Refreshes all cached signals, at most once per robot loop."""
        now = Timer.getFPGATimestamp()
        if now - self._lastRefresh < 0.005:
            return
        self._lastRefresh = now
        status = BaseStatusSignal.refresh_all(
            self._yawSignal, self._yawRateSignal, self._pitchSignal, self._rollSignal)
        self._connected = status.is_ok()
        self._rawYaw = self._yawSignal.value
        self._yawRate = self._yawRateSignal.value
        self._pitch = self._pitchSignal.value
        self._roll = self._rollSignal.value

    def _periodic(self) -> None:
        self._refresh()
        self._updateTelemetry()

    def isConnected(self) -> bool:
        """True if the last signal refresh came back OK."""
        self._refresh()
        return self._connected

    def getHeading(self) -> units.degrees:
        self._refresh()
        return utils.wrapAngle(self._rawYaw + self._angleAdjustment)

    def getRotation2d(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.getHeading())

    def getPitch(self) -> units.degrees:
        self._refresh()
        return self._pitch

    def getRoll(self) -> units.degrees:
        self._refresh()
        return self._roll

    def getYawRate(self) -> units.degrees_per_second:
        self._refresh()
        return self._yawRate

    def _reset(self, heading: units.degrees = 0) -> None:
        self._angleAdjustment = -heading if heading != 0 else 0
        self._gyro.reset()
        # Force the cached values to be re-read on the next access.
        self._lastRefresh = -1.0

    def resetRobotToField(self, robotPose: Pose2d) -> None:
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            offset_angle = 0.0

        else:
            offset_angle = 180.0
        
        self._reset(utils.wrapAngle(robotPose.rotation().degrees() + offset_angle))

    def reset(self) -> Command:
        return cmd.runOnce(self._reset).withName("GyroSensor:Reset")

    def _updateTelemetry(self) -> None:
        SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
        SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
        SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
        SmartDashboard.putNumber(f'{self._baseKey}/YawRate', self.getYawRate())
        SmartDashboard.putBoolean(f'{self._baseKey}/Connected', self._connected)



# class Gyro_NAVX2():
#     def __init__(
#         self,
#         comType: AHRS.NavXComType
#     ) -> None:
#         self._baseKey = f'Robot/Sensors/Gyro'

#         self._gyro = AHRS(comType)

#         self._angleAdjustment: units.degrees = 0

#         utils.addRobotPeriodic(self._periodic)

#     def _periodic(self) -> None:
#         self._updateTelemetry()

#     def getHeading(self) -> units.degrees:
#         return -utils.wrapAngle(self._gyro.getAngle() + self._angleAdjustment)

#     def getPitch(self) -> units.degrees:
#         return self._gyro.getPitch()

#     def getRoll(self) -> units.degrees:
#         return self._gyro.getRoll()

#     def _reset(self, heading: units.degrees = 0) -> None:
#         self._angleAdjustment = -heading if heading != 0 else 0
#         self._gyro.reset()

#     def resetRobotToField(self, robotPose: Pose2d) -> None:
#         if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
#             offset_angle = 0.0
#         else:
#             offset_angle = 180.0

#         self._reset(utils.wrapAngle(robotPose.rotation().degrees() + offset_angle))

#     def reset(self) -> Command:
#         return cmd.runOnce(self._reset).withName("GyroSensor:Reset")

#     def _updateTelemetry(self) -> None:
#         SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
#         SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
#         SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
