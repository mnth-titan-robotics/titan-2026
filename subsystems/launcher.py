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
            rev.SparkBase.MotorType.kBrushless
        )
        self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        self._motor.applySparkMaxConfig(spark_config)