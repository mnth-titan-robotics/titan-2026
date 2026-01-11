from commands2 import Subsystem, Command
from typing import Callable
import constants
import rev

Constants = constants.Subsystems.Roller


class Roller(Subsystem):
    """Runs the roller, ejecting coral"""

    def __init__(self):
        self._motor = rev.SparkMax(
            Constants.MotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig() \
            .inverted(True) \
            .voltageCompensation(Constants.MotorVComp) \
            .smartCurrentLimit(Constants.MotorCurrentLimit)
        self._motor.configure(
            spark_config,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters)

    def stopCommand(self) -> Command:
        return self.runOnce(self._motor.stopMotor)

    def ejectCommand(self) -> Command:
        return self.run(
            lambda: self._motor.set(Constants.EjectSpeed)
        ).andThen(self.stopCommand())

    def reverseCommand(self) -> Command:
        return self.run(
            lambda: self._motor.set(Constants.ReverseSpeed)
        ).andThen(self.stopCommand())
