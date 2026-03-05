import rev
from commands2 import Subsystem, Command
import ntcore
import constants


class Launcher(Subsystem):
    """Controls the launcher mechanism for firing projectiles."""

    Constants = constants.Subsystems.Launcher()

    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic("Subsystems/Launcher/Launcher_Speed").getEntry(self.Constants.LaunchSpeed)
        self._speed_entry.setDefault(self.Constants.LaunchSpeed)

        self._leftMotor = rev.SparkMax(
            self.Constants.LeftMotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._rightMotor = rev.SparkMax(
            self.Constants.RightMotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._leftMotor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        spark_config.inverted(False)
        spark_config.smartCurrentLimit(self.Constants.MotorCurrentLimit)
        spark_config.voltageCompensation(self.Constants.MotorVComp)
        self._leftMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        spark_config.follow(self.Constants.LeftMotorId,True)
        self._rightMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._encoder = self._leftMotor.getEncoder()

    def start(self) -> Command:
        """Starts the launcher at launch speed."""

        def command_function():
            self._leftMotor.set(self._speed_entry.get())

        return self.run(command_function).withName("LauncherStart")

    def stop(self) -> Command:
        """Stops the launcher."""

        def command_function():
            self._leftMotor.set(0)

        return self.run(command_function).withName("LauncherStop")

    def at_speed(self) -> bool:
        """Returns True if the launcher is at launch speed."""
        return abs(self._encoder.getVelocity()) >= self.Constants.MinLaunchSpeed
