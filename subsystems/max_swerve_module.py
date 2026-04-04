#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from rev import SparkMax, SparkFlex, SparkBase, ResetMode, PersistMode
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState

from configs import Configs


class MAXSwerveModule:
    def __init__(self, drivingCANId: int, turningCANId: int, chassisAngularOffset: float):
        """
        Constructs a MAXSwerveModule and configures the driving and turning motor,
        encoder, and PID controller. This configuration is specific to the REV
        MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
        Encoder. 
        """
        self._drivingSpark = SparkFlex(drivingCANId, SparkBase.MotorType.kBrushless)
        self._turningSpark = SparkMax(turningCANId, SparkBase.MotorType.kBrushless)

        self._drivingEncoder = self._drivingSpark.getEncoder()
        self._turningEncoder = self._turningSpark.getAbsoluteEncoder()

        self._drivingClosedLoopController = self._drivingSpark.getClosedLoopController()
        self._turningClosedLoopController = self._turningSpark.getClosedLoopController()

        # Apply the respective configurations to the SPARKS. Reset parameters before
        # applying the configuration to bring the SPARK to a known good state. Persist
        # the settings to the SPARK to avoid losing them on a power cycle.
        self._drivingSpark.configure(Configs.MAXSwerveModule.kDrivingConfig, ResetMode.kResetSafeParameters,
                                     PersistMode.kPersistParameters)
        self._turningSpark.configure(Configs.MAXSwerveModule.kTurningConfig, ResetMode.kResetSafeParameters,
                                     PersistMode.kPersistParameters)

        self._chassisAngularOffset = chassisAngularOffset
        self._desiredState = SwerveModuleState(0, Rotation2d(self._turningEncoder.getPosition()))
        self._drivingEncoder.setPosition(0)

    def getState(self) -> SwerveModuleState:
        """
        Returns the current state of the module.
        :return: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        return SwerveModuleState(self._drivingEncoder.getVelocity(),
                                 Rotation2d(self._turningEncoder.getPosition() - self._chassisAngularOffset))

    def getPosition(self) -> SwerveModulePosition:
        """
        Returns the current position of the module.
        :return: The current position of the module. 
        """
        # Apply chassis angular offset to the encoder position to get the position
        return SwerveModulePosition(
            self._drivingEncoder.getPosition(),
            Rotation2d(self._turningEncoder.getPosition() - self._chassisAngularOffset))

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """
        Sets the desired state for the module.
        :param desiredState: Desired state with speed and angle.
        """
        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self._chassisAngularOffset)

        # Optimize the reference state to avoid spinning further than 90 degrees.
        correctedDesiredState.optimize(Rotation2d(self._turningEncoder.getPosition()))

        # Command driving and turning SPARKS towards their respective setpoints.
        self._drivingClosedLoopController.setReference(correctedDesiredState.speed, SparkMax.ControlType.kVelocity)
        self._turningClosedLoopController.setReference(correctedDesiredState.angle.radians(), SparkMax.ControlType.kPosition)

        self._desiredState = desiredState

    def getDesiredState(self) -> SwerveModuleState:
        return self._desiredState

    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        self._drivingEncoder.setPosition(0)
