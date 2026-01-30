#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib
from random import gauss
from wpilib import RobotController
from wpilib.simulation import ADIS16470_IMUSim
from wpimath import units
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.interpolation import TimeInterpolatablePose2dBuffer
from pyfrc.physics.core import PhysicsInterface
from services.questnav.questnav_data import PoseFrame
from services.questnav.questnav_stub import QuestNavStub
from robot import Robot
from sim.swerve_drive_sim import SwerveDriveSim
import wpimath

class QuestNavSim:
    def __init__(self, physics_controller: PhysicsInterface, questnav: QuestNavStub):
        self.field = physics_controller.field
        self._questnav = questnav
        self._poseBuffer = TimeInterpolatablePose2dBuffer(1.0)
        self.frame_number = 1

    def simulationPeriodic(self, dt: float) -> None:
        pose = self.field.getRobotPose()
        t = wpilib.Timer.getFPGATimestamp()
        self._poseBuffer.addSample(t, pose)
        latency = gauss(0.01, 0.005)
        t -= latency
        delayed_pose = self._poseBuffer.sample(t)
        rotation3d = Rotation3d(units.radians(0.0), units.radians(0.0), delayed_pose.rotation().radians())
        pose3d = Pose3d(delayed_pose.x, delayed_pose.y, 0.0, rotation3d)
        self.frame_number += 1
        self._questnav.unread_frames.append(PoseFrame(pose3d, t, t, self.frame_number))


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: Robot):
        self._robot = robot
        self.physics_controller = physics_controller
        self._questNavSim = QuestNavSim(physics_controller, robot.container._localization._questnav)
        self._driveSim = SwerveDriveSim(robot.container._drive)
        self._gyroSim = ADIS16470_IMUSim(robot.container._drive._gyro)

    def update_sim(self, now: float, tm_diff: float) -> None:
        vBus = RobotController.getBatteryVoltage()
        self._questNavSim.simulationPeriodic(tm_diff)
        if self._robot.isEnabled():
            chassisSpeeds = self._driveSim.simulationPeriodic(tm_diff)
            self.physics_controller.drive(chassisSpeeds, tm_diff)
            pose = self.physics_controller.get_pose()

            # Update the gyro sim
            # BUG: degrees() is returning radians for some reason. Things are weird.
            rot = pose.rotation().degrees()
            self._gyroSim.setGyroAngleZ(rot)
        else:
            pass
