from commands2 import Subsystem, Command
from typing import Callable
import constants
import rev

class Launcher(Subsystem):
    """Controls the launcher mechanism for firing projectiles."""

    Constants = constants.Subsystems.Launcher()

    def __init__(self):
        self._motor = rev.SparkMax(
            self.Constants.MotorId,
            rev.SparkBase.MotorType.kBrushed
        )
        self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        spark_config.inverted(True)
        self._motor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

    def start(self) -> Command:
        """Starts the launcher at launch speed."""
        def command_function():
            self._motor.set(self.Constants.LaunchSpeed)
        return self.run(command_function).withName("LauncherStart")
    
    def stop(self) -> Command:
        """Stops the launcher."""
        def command_function():
            self._motor.set(0)
        return self.run(command_function).withName("LauncherStop")