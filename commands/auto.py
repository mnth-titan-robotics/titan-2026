from typing import TYPE_CHECKING
from wpimath.kinematics import ChassisSpeeds
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard

if TYPE_CHECKING:
    from robotcontainer import RobotContainer


class Auto:
    def __init__(
            self,
            robot: "RobotContainer"
    ) -> None:
        self._robot = robot

        self._selectedAuto = cmd.none()

        # Add Robot/Auto to SmartDashboard with options
        self._autos = SendableChooser()
        # Default 'None' auto, does nothing. The second parameter is a function that will be called in our onChange callback below
        self._autos.setDefaultOption("None", cmd.none)
        self._autos.addOption("[Center]", self.auto_center)
        # When the selected auto changes call the function (eg: cmd.none(), self.auto_center()) to get the command, then store it in self._auto
        self._autos.onChange(lambda auto: self.set(auto()))
        # Send the list of options to SmartDashboard
        SmartDashboard.putData("Robot/Auto", self._autos)

    def get(self) -> Command:
        return self._selectedAuto

    def set(self, autoCmd: Command) -> None:
        self._selectedAuto = autoCmd

    def auto_center(self) -> Command:
        # Move forward at 25% speed for 3.25s, then stop
        drive = self._robot._drive
        roller = self._robot._roller
        speeds = ChassisSpeeds(vx=0.25)
        return cmd.sequence(
            drive.driveCommand(speeds).withTimeout(3.25),
            drive.stopCommand().withTimeout(0.1),
            roller.ejectCommand().withTimeout(0.5)
        )
