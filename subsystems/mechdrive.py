from typing import Callable
from wpimath.geometry import Rotation2d, Pose2d, Transform2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveModulePosition, SwerveDrive4Odometry, SwerveDrive4Kinematics
from wpimath import units
from commands2 import Subsystem, Command
from .max_swerve_module import MAXSwerveModule
from wpilib import ADIS16470_IMU
from ntcore import NetworkTableInstance
from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SendableChooser
from wpilib.drive import MecanumDrive
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, MecanumDriveWheelSpeeds, MecanumDriveWheelPositions
from lib import utils
from lib.classes import DifferentialModuleLocation, DifferentialDriveModulePositions, MotorIdleMode, SpeedMode, DriveOrientation, OptionState, TargetAlignmentMode
from lib.components.differential_module import DifferentialModule
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants



import constants

IMUAxis = ADIS16470_IMU.IMUAxis
DriveConstants = constants.Subsystems.Drive
class DriveSubsystem(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._differentialModules = dict((c.location, DifferentialModule(c)) for c in self._constants.kDifferentialModuleConfigs)
    
    self._drivetrain = MecanumDrive(
      self._differentialModules[DifferentialModuleLocation.LeftFront].getMotorController(),
      self._differentialModules[DifferentialModuleLocation.LeftRear].getMotorController(),
      self._differentialModules[DifferentialModuleLocation.RightFront].getMotorController(),
      self._differentialModules[DifferentialModuleLocation.RightRear].getMotorController()
    )

    self._isDriftCorrectionActive: bool = False
    self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
    self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._isAlignedToTarget: bool = False
    self._targetAlignmentRotationController = PIDController(*self._constants.kTargetAlignmentConstants.rotationPID)
    self._targetAlignmentRotationController.setTolerance(*self._constants.kTargetAlignmentConstants.rotationTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)
    self._targetAlignmentTranslationXController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationXController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationXController.setSetpoint(0)
    self._targetAlignmentTranslationYController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationYController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationYController.setSetpoint(0)
    self._targetPose: Pose3d = None

    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedModeChooser = SendableChooser()
    speedModeChooser.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedModeChooser.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    #SmartDashboard.putData("Robot/Drive/SpeedMode", speedModeChooser)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientationChooser = SendableChooser()
    orientationChooser.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientationChooser.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientationChooser.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    #SmartDashboard.putData("Robot/Drive/Orientation", orientationChooser)

    self._driftCorrection: OptionState = OptionState.Enabled
    driftCorrectionChooser = SendableChooser()
    driftCorrectionChooser.setDefaultOption(OptionState.Enabled.name, OptionState.Enabled)
    driftCorrectionChooser.addOption(OptionState.Disabled.name, OptionState.Disabled)
    driftCorrectionChooser.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    #SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrectionChooser)

    idleModeChooser = SendableChooser()
    idleModeChooser.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleModeChooser.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleModeChooser.onChange(lambda idleMode: self._setIdleMode(idleMode))
    #SmartDashboard.putData("Robot/Drive/IdleMode", idleModeChooser)

    #SmartDashboard.putNumber("Robot/Drive/Chassis/Length", self._constants.kWheelBase)
    #SmartDashboard.putNumber("Robot/Drive/Chassis/Width", self._constants.kTrackWidth)
    #SmartDashboard.putNumber("Robot/Drive/Speed/Max", self._constants.kTranslationSpeedMax)

  def periodic(self) -> None:
    self._updateTelemetry()

  def driveCommand(
      self, 
      getInputX: Callable[[], float], 
      getInputY: Callable[[], float], 
      getInputRotation: Callable[[], float]
    ) -> Command:
    return self.run(
      lambda: self._drive(getInputX(), getInputY(), getInputRotation())
    ).withName("DriveSubsystem:Drive")

  def _drive(self, inputX: float, inputY: float, inputRotation: float) -> None:
    #RateLimit = 0.5
    #SRL = SlewRateLimiter(rateLimit=RateLimit)

    #inputRotation_Limited = SRL.calculate(inputRotation)
    # Theoretically ^ this ^ should be input scaling
    # This causes issues with rotation where it just doesn't so idk

    #chassisSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(
    #  inputX, 
    #  inputY, 
    #  inputRotation, 
    #  Rotation2d()  #(.fromDegrees(self._getGyroHeading()))
    #)
    #self.drive(chassisSpeeds=chassisSpeed)

    self._drivetrain.driveCartesian(
      xSpeed=inputX,
      ySpeed=inputY,
      zRotation=inputRotation,
      gyroAngle=Rotation2d()  #.fromDegrees(self._getGyroHeading())
    )

  def drive(self, chassisSpeeds: ChassisSpeeds) -> None:
    wheelSpeeds = self._constants.kDriveKinematics.toWheelSpeeds(chassisSpeeds)
    
    self._differentialModules[DifferentialModuleLocation.LeftFront].setVelocity(wheelSpeeds.frontLeft)
    self._differentialModules[DifferentialModuleLocation.RightFront].setVelocity(wheelSpeeds.frontRight)
    self._differentialModules[DifferentialModuleLocation.LeftRear].setVelocity(wheelSpeeds.rearLeft)
    self._differentialModules[DifferentialModuleLocation.RightRear].setVelocity(wheelSpeeds.rearRight)

    # TODO: Fix this, or get rid of this method
    # self._drivetrain.tankDrive(wheelSpeeds.left, wheelSpeeds.right)
    self.clearTargetAlignment()

  def getModulePositions(self) -> MecanumDriveWheelPositions:
    wheel_positions = MecanumDriveWheelPositions()
    wheel_positions.frontLeft = self._differentialModules[DifferentialModuleLocation.LeftFront].getPosition()
    wheel_positions.frontRight = self._differentialModules[DifferentialModuleLocation.RightFront].getPosition()
    wheel_positions.rearLeft = self._differentialModules[DifferentialModuleLocation.LeftRear].getPosition()
    wheel_positions.rearRight = self._differentialModules[DifferentialModuleLocation.RightRear].getPosition()

    return wheel_positions

  def getChassisSpeeds(self) -> ChassisSpeeds:
    wheel_velocities = MecanumDriveWheelSpeeds()
    wheel_velocities.frontLeft = self._differentialModules[DifferentialModuleLocation.LeftFront].getVelocity()
    wheel_velocities.frontRight = self._differentialModules[DifferentialModuleLocation.RightFront].getVelocity()
    wheel_velocities.rearLeft = self._differentialModules[DifferentialModuleLocation.LeftRear].getVelocity()
    wheel_velocities.rearRight = self._differentialModules[DifferentialModuleLocation.RightRear].getVelocity()
    self._constants.kDriveKinematics.toWheelSpeeds(
      
    )
    return self._constants.kDriveKinematics.toChassisSpeeds(wheel_velocities)

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    # TODO: implement idleMode change on motor controllers
    #SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)
    pass

  def alignToTargetCommand(
      self, 
      getRobotPose: Callable[[], Pose2d], 
      getTargetPose: Callable[[TargetAlignmentLocation], Pose3d], 
      targetAlignmentMode: TargetAlignmentMode, 
      targetAlignmentLocation: TargetAlignmentLocation,
      targetType: TargetType
    ) -> Command:
    return self.run(
      lambda: self._runTargetAlignment(getRobotPose(), targetAlignmentMode)
    ).beforeStarting(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(targetAlignmentLocation, targetType), targetAlignmentMode)
    ).until(
      lambda: self._isAlignedToTarget
    ).withName("DriveSubsystem:AlignToTarget")
  
  def _initTargetAlignment(
      self, 
      robotPose: Pose2d, 
      targetPose: Pose3d, 
      targetAlignmentMode: TargetAlignmentMode
    ) -> None:
    self.clearTargetAlignment()
    self._targetPose = targetPose
    self._targetAlignmentRotationController.reset()
    if targetAlignmentMode == TargetAlignmentMode.Heading:
      self._targetAlignmentRotationController.setSetpoint(utils.wrapAngle(utils.getTargetHeading(robotPose, targetPose) + self._constants.kTargetAlignmentConstants.rotationHeadingModeOffset))
    else:
      self._targetAlignmentRotationController.setSetpoint(targetPose.toPose2d().rotation().degrees() + self._constants.kTargetAlignmentConstants.rotationTranslationModeOffset)
    self._targetAlignmentTranslationXController.reset()
    self._targetAlignmentTranslationYController.reset()
    
  def _runTargetAlignment(self, robotPose: Pose2d, targetAlignmentMode: TargetAlignmentMode) -> None:
    targetTranslation = self._targetPose.__sub__(Pose3d(robotPose))

    speedRotation = 0
    speedTranslationX = 0
    speedTranslationY = 0

    # TODO: implement differential drive target alignment logic and drive values

    # if not self._targetAlignmentRotationController.atSetpoint():
    #   speedRotation = self._targetAlignmentRotationController.calculate(robotPose.rotation().degrees())

    # if targetAlignmentMode == TargetAlignmentMode.Translation and not self._targetAlignmentTranslationXController.atSetpoint():
    #   speedTranslationX = self._targetAlignmentTranslationXController.calculate(targetTranslation.X())

    # if targetAlignmentMode == TargetAlignmentMode.Translation and not self._targetAlignmentTranslationYController.atSetpoint():
    #   speedTranslationY = self._targetAlignmentTranslationYController.calculate(targetTranslation.Y())

    # self._setSwerveModuleStates(
    #   self._constants.kDriveKinematics.toSwerveModuleStates(
    #     ChassisSpeeds(
    #       -utils.clampValue(speedTranslationX, -self._constants.kTargetAlignmentConstants.translationSpeedMax, self._constants.kTargetAlignmentConstants.translationSpeedMax), 
    #       -utils.clampValue(speedTranslationY, -self._constants.kTargetAlignmentConstants.translationSpeedMax, self._constants.kTargetAlignmentConstants.translationSpeedMax),
    #       utils.clampValue(speedRotation, -self._constants.kTargetAlignmentConstants.rotationSpeedMax, self._constants.kTargetAlignmentConstants.rotationSpeedMax)
    #     )
    #   )
    # )

    if speedRotation == 0 and speedTranslationX == 0 and speedTranslationY == 0:
      self._isAlignedToTarget = True

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def reset(self) -> None:
    self._drive(0.0, 0.0, 0.0)
    self.clearTargetAlignment()
  
  def _updateTelemetry(self) -> None:
    pass
  
class MechDrive:
    pass
    def periodic(self) -> None:
        # self._odometry.update(
        #   Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
        #   (
        #     self._frontLeft.getPosition(),
        #     self._frontRight.getPosition(),
        #     self._rearLeft.getPosition(),
        #     self._rearRight.getPosition()
        #   )
        # )
        self.updateTelemetry()

    def updateTelemetry(self) -> None:
        self._publisher.set(self.getPose())

        # self._statePublisher.set([
        #   self._frontLeft.getState(),
        #   self._frontRight.getState(),
        #   self._rearLeft.getState(),
        #   self._rearRight.getState()]
        # )

        # self._desiredStatePublisher.set([
        #   self._frontLeft.getDesiredState(),
        #   self._frontRight.getDesiredState(),
        #   self._rearLeft.getDesiredState(),
        #   self._rearRight.getDesiredState()
        # ])

    def getPose(self) -> Pose2d:
        """
        Returns the currently-estimated pose of the robot.
        :return: The pose.
        """
        return self._odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """
        Resets the odometry to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        # self._odometry.resetPosition(
        #   Rotation2d.fromDegrees(self._gyro.getAngle(IMUAxis.kZ)),
        #   (
        #     self._frontLeft.getPosition(),
        #     self._frontRight.getPosition(),
        #     self._rearLeft.getPosition(),
        #     self._rearRight.getPosition()
        #   ),
        #   pose)
        
    def driveCommand(
                self,
                get_input: Callable[[], ChassisSpeeds]) -> Command:
            """Returns a command that drives the robot"""
            return self.run(
            # TODO: If "locked on" ignore the input rotation, calculate desired rotation to point at the target
                lambda: self.setChassisSpeed(get_input())
            )
    
    def lockOnCommand(self, target_pose: Pose2d) -> Command:
        pass

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool) -> None:
        pass

    def setChassisSpeed(self, chassisSpeeds: ChassisSpeeds) -> None:
        #TODO ALL IMPLEMENT 
        pass

    def autoDrive(self, xOffset: units.inches, yOffset: units.inches, rotation: units.degrees):
        pass

    def setX(self) -> None:
        """
        Sets the wheels into an X formation to prevent movement.
        """
        # self._frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        # self._frontRight.setDesiredState((0, Rotation2d.fromDegrees(-45)))
        # self._rearLeft.setDesiredState((0, Rotation2d.fromDegrees(-45)))
        # self._rearRight.setDesiredState((0, Rotation2d.fromDegrees(45)))

    def setModuleStates(self, desiredStates: tuple[SwerveModuleState, ...]) -> None:
        pass

    def resetEncoders(self) -> None:
        """
        Resets the drive encoders to currently read a position of 0.
        """
        # self._frontLeft.resetEncoders()
        # self._rearLeft.resetEncoders()
        # self._frontRight.resetEncoders()
        # self._rearRight.resetEncoders()

    def zeroHeading(self) -> None:
        pass
    def getHeading(self) -> float:
        pass

    def getTurnRate(self) -> float:
        pass