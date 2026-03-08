#
# I stole this straight from GitHub - Logan
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math

from rev import SparkMaxConfig, ClosedLoopConfig, SparkBaseConfig, FeedbackSensor
from constants import ModuleConstants


class Configs:
  class MAXSwerveModule:
    kDrivingConfig: SparkMaxConfig = SparkMaxConfig().inverted(False)
    kTurningConfig: SparkMaxConfig = SparkMaxConfig().inverted(False)

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