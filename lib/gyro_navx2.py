from commands2 import Command, cmd
from wpilib import DriverStation, RobotBase, SmartDashboard
from wpimath import units
from wpimath.geometry import Pose2d
from navx import AHRS
from lib import logger, utils


class Gyro_NAVX2():
    def __init__(
        self,
        comType: AHRS.NavXComType
    ) -> None:
        self._baseKey = f'Robot/Sensors/Gyro'

        self._gyro = AHRS(comType)

        self._angleAdjustment: units.degrees = 0

        utils.addRobotPeriodic(self._periodic)

    def _periodic(self) -> None:
        self._updateTelemetry()

    def getHeading(self) -> units.degrees:
        return -utils.wrapAngle(self._gyro.getAngle() + self._angleAdjustment)

    def getPitch(self) -> units.degrees:
        return self._gyro.getPitch()

    def getRoll(self) -> units.degrees:
        return self._gyro.getRoll()

    def _reset(self, heading: units.degrees = 0) -> None:
        self._angleAdjustment = -heading if heading != 0 else 0
        self._gyro.reset()

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
