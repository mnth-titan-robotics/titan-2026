from commands2 import Subsystem, Command
from typing import Callable
import constants
import rev

class Climber(Subsystem):
    """Controls the climbing mechanism for ascending structures."""

    Constants = constants.Subsystems.Climber()

    def __init__(self):
        self._motor = rev.SparkMax(
            self.Constants.MotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        self._motor.applySparkMaxConfig(spark_config)

    def climb(self) -> Command:
        """Starts the climber at climb speed."""
        def command_function():
            pass
        return self.run(command_function).withName("ClimberStart")

    def descend(self) -> Command:
        """Reverses the climber."""
        def command_function():
            pass
        return self.run(command_function).withName("ClimberDescend")