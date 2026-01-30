import rev
from commands2 import Subsystem, Command

import constants


class Intake(Subsystem):
    """Controls the intake mechanism for collecting game pieces."""

    Constants = constants.Subsystems.Intake()

    def __init__(self):
        self._motor = rev.SparkMax(
            self.Constants.MotorId,
            rev.SparkBase.MotorType.kBrushed
        )
        self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        self._motor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

    def intake(self) -> Command:
        """Starts the intake at intake speed."""

        def command_function():
            self._motor.set(self.Constants.IntakeSpeed)

        return self.run(command_function).withName("IntakeStart")

    def reverse(self) -> Command:
        """Reverses the intake."""

        def command_function():
            self._motor.set(self.Constants.ReverseSpeed)

        return self.run(command_function).withName("IntakeReverse")

    def stop(self) -> Command:
        """Stops the intake."""

        def command_function():
            self._motor.set(0)

        return self.run(command_function).withName("IntakeStop")

    def extend(self) -> Command:
        """Extends the intake mechanism."""

        def command_function():
            # Placeholder for extension logic
            pass

        return self.run(command_function).withName("IntakeExtend")

    def retract(self) -> Command:
        """Retracts the intake mechanism."""

        def command_function():
            # Placeholder for retraction logic
            pass

        return self.run(command_function).withName("IntakeRetract")
