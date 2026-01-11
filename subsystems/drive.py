from commands2 import Subsystem, Command
from typing import Callable
from wpilib.drive import MecanumDrive
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveWheelSpeeds, MecanumDriveOdometry, MecanumDriveWheelPositions

from lib.enums import ModuleLocation
from lib.differential_module import DifferentialModule
import constants

Constants = constants.Subsystems.Drive


class Drive(Subsystem):
    """Responsible for moving our robot"""

    def __init__(self):
        # Calls the Subsystem __init__ method
        super().__init__()
        self._differentialModules = dict((c.location, DifferentialModule(c))
                                         for c in Constants.ModuleConfigs)
        self._kinematics = Constants.Kinematics
        self._drivetrain = MecanumDrive(
            self._differentialModules[ModuleLocation.LeftFront].getMotorController(),
            self._differentialModules[ModuleLocation.LeftRear].getMotorController(),
            self._differentialModules[ModuleLocation.RightFront].getMotorController(),
            self._differentialModules[ModuleLocation.RightRear].getMotorController()
        )

        # TODO: Get initial pose from a localization system
        self._odometry = MecanumDriveOdometry(
            self._kinematics,
            gyroAngle=self.getHeading(),
            wheelPositions=self.getWheelPositions(),
            initialPose=Pose2d()
        )
    
    def getHeading(self) -> Rotation2d:
        """Gets the robot's current heading (typically from a gyro)"""
        return Rotation2d()

    def periodic(self) -> None:
        # Update the odometry in the periodic block
        self._odometry.update(
            Rotation2d(),
            self.getWheelPositions()
        )
    
    def getPose(self) -> Pose2d:
        """Returns the current robot pose based on odometry"""
        return self._odometry.getPose()
    
    def resetOdometry(self, pose: Pose2d) -> None:
        self._odometry.resetPosition(
            gyroAngle=self.getHeading(),
            wheelPositions=self.getWheelPositions(),
            pose=pose
        )

    def driveCommand(
            self,
            get_input: Callable[[], ChassisSpeeds]) -> Command:
        """Returns a command that drives the robot"""
        return self.run(
            lambda: self.setChassisSpeed(get_input())
        )

    def stopCommand(self) -> Command:
        """Returns a command that stops the robot"""
        return self.run(
            lambda: self.setChassisSpeed(ChassisSpeeds())
        )

    def setWheelSpeeds(self, wheelSpeeds: MecanumDriveWheelSpeeds) -> None:
        chassisSpeeds = self._kinematics.toChassisSpeeds(wheelSpeeds)
        self.setChassisSpeed(chassisSpeeds)

    def setChassisSpeed(self, chassisSpeeds: ChassisSpeeds) -> None:
        self._drivetrain.driveCartesian(
            xSpeed=chassisSpeeds.vx,
            ySpeed=chassisSpeeds.vy,
            zRotation=chassisSpeeds.omega,
            gyroAngle=self.getHeading()
        )

    def getChassisSpeeds(self) -> ChassisSpeeds:
        """Gets the velocity of the robot based solely on wheel odometry"""
        wheelSpeeds = MecanumDriveWheelSpeeds()
        wheelSpeeds.frontLeft = self._differentialModules[ModuleLocation.LeftFront].getVelocity()
        wheelSpeeds.frontRight = self._differentialModules[ModuleLocation.RightFront].getVelocity()
        wheelSpeeds.rearLeft = self._differentialModules[ModuleLocation.LeftRear].getVelocity()
        wheelSpeeds.rearRight = self._differentialModules[ModuleLocation.RightRear].getVelocity()
        self._kinematics.toChassisSpeeds(wheelSpeeds)

    def getWheelPositions(self) -> MecanumDriveWheelPositions:
        wheelPositions = MecanumDriveWheelPositions()
        wheelPositions.frontLeft = self._differentialModules[ModuleLocation.LeftFront].getPosition()
        wheelPositions.frontRight = self._differentialModules[ModuleLocation.RightFront].getPosition()
        wheelPositions.rearLeft = self._differentialModules[ModuleLocation.LeftRear].getPosition()
        wheelPositions.rearRight = self._differentialModules[ModuleLocation.RightRear].getPosition()
        return wheelPositions
