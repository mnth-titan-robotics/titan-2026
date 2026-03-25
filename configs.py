#
# I stole this straight from GitHub - Logan
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from rev import SparkMaxConfig, SparkBaseConfig, FeedbackSensor
from constants import ModuleConstants, Subsystems


class Configs:
    class Indexer:
        kConfig: SparkBaseConfig = SparkMaxConfig().inverted(True) \
            .voltageCompensation(Subsystems.Indexer.MotorVComp) \
            .smartCurrentLimit(Subsystems.Indexer.MotorCurrentLimit)

    class Intake:
        kConfig: SparkBaseConfig = SparkMaxConfig().inverted(False)

    class IntakeExtender:
        kLeftConfig: SparkBaseConfig = SparkMaxConfig().inverted(True) \
            .voltageCompensation(Subsystems.IntakeExtender.MotorVComp)
        kLeftConfig.smartCurrentLimit(Subsystems.IntakeExtender.MotorCurrentLimit)
        kLeftConfig.closedLoop \
            .pid(*Subsystems.IntakeExtender.PID)
        kLeftConfig.closedLoop.maxMotion \
            .cruiseVelocity(Subsystems.IntakeExtender.MotorSpeed) \
            .maxAcceleration(Subsystems.IntakeExtender.MaxAcceleration) \
            .allowedProfileError(Subsystems.IntakeExtender.AllowedProfileError)
        kLeftConfig.encoder \
            .positionConversionFactor(Subsystems.IntakeExtender.GearReduction) \
            .velocityConversionFactor(Subsystems.IntakeExtender.GearReduction / 60.0)

        kRightConfig: SparkBaseConfig = SparkMaxConfig().apply(kLeftConfig) \
            .follow(Subsystems.IntakeExtender.LeftMotorId, True)

    class Launcher:
        kLeftConfig: SparkBaseConfig = SparkMaxConfig().inverted(True)
        kLeftConfig.smartCurrentLimit(Subsystems.Launcher.MotorCurrentLimit)
        kLeftConfig.voltageCompensation(Subsystems.Launcher.MotorVComp)
        kLeftConfig.closedLoop \
            .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder) \
            .pidf(0.33, 0.0, 0.0, 0.3)
        kLeftConfig.encoder \
            .positionConversionFactor(Subsystems.Launcher.PositionConversionFactor) \
            .velocityConversionFactor(Subsystems.Launcher.VelocityConversionFactor)

        # Configure the right motor to follow the left motor with inverted output, since they are mounted in opposite directions.
        kRightConfig: SparkBaseConfig = SparkMaxConfig().apply(kLeftConfig) \
            .follow(Subsystems.Launcher.LeftMotorId, True)

    class MAXSwerveModule:
        kDrivingConfig: SparkBaseConfig = SparkMaxConfig().inverted(False)
        kTurningConfig: SparkBaseConfig = SparkMaxConfig().inverted(False)

        # Use module constants to calculate conversion factors and feed forward gain.
        drivingFactor = ModuleConstants.kWheelDiameterMeters * math.pi / ModuleConstants.kDrivingMotorReduction
        turningFactor = 2 * math.pi
        drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps
        (kDrivingConfig
         .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
         .smartCurrentLimit(50)
         )
        (kDrivingConfig.encoder
         .positionConversionFactor(drivingFactor)  # meters
         .velocityConversionFactor(drivingFactor / 60.0)  # meters per second
         )
        (kDrivingConfig.closedLoop
         .setFeedbackSensor(FeedbackSensor.kPrimaryEncoder)
         # These are example gains - you may need to modify them for your own robot!
         .pid(0.04, 0, 0)
         .velocityFF(drivingVelocityFeedForward)
         .outputRange(-1, 1)
         )

        (kTurningConfig
         .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
         .smartCurrentLimit(20)
         )
        (kTurningConfig.absoluteEncoder
         # Invert the turning encoder, since the output shaft rotates in the opposite
         # direction of the steering motor in the MAXSwerve Module.
         .inverted(True)
         .positionConversionFactor(turningFactor)  # radians
         .velocityConversionFactor(turningFactor / 60.0)  # radians per second
         )
        (kTurningConfig.closedLoop
         .setFeedbackSensor(FeedbackSensor.kAbsoluteEncoder)
         # These are example gains - you may need to modify them for your own robot!
         .pid(1, 0, 0)
         .outputRange(-1, 1)
         # Enable PID wrap around for the turning motor. This will allow the PID
         # controller to go through 0 to get to the setpoint i.e. going from 350 degrees
         # to 10 degrees will go through 0 rather than the other direction which is a
         # longer route.
         .positionWrappingEnabled(True)
         .positionWrappingInputRange(0, turningFactor)
         )
