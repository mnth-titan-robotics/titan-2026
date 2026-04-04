import rev
from commands2 import Subsystem, Command
import ntcore
from configs import Configs
import constants
from wpilib import RobotBase, RobotController
from wpimath import units
from services import Tracker

ENABLE_TELEMETRY = constants.ENABLE_TELEMETRY


class Launcher(Subsystem):
    """Controls the launcher mechanism for firing projectiles."""

    Constants = constants.Subsystems.Launcher()
    # 3200 RPM - 9'  6" 1.26s
    # 3500 RPM -
    #   12' 2" 1.32s
    #   14' 6" 1.51s
    #
    # 3.8 - corner - 5.2m
    # 3.3 - mid - ~4.5m
    # 2.8 - trench - ~3.5m
    SPEED_LOOKUP_SLOPE = (3.8 - 2.8) / (5.2 - 3.5)
    SPEED_LOOKUP_INTERCEPT = 3.8 - SPEED_LOOKUP_SLOPE * 5.2

    def __init__(self, tracker: Tracker):
        self._tracker = tracker
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic(
            "Subsystems/Launcher/Launcher_Speed").getEntry(self.Constants.LaunchSpeed)
        self._speed_entry.setDefault(self.Constants.LaunchSpeed)
        if ENABLE_TELEMETRY:
            self._cur_speed = self.nt_instance.getFloatTopic("Subsystems/Launcher/Launcher_Velocity").publish()
            self._cur_amps = self.nt_instance.getFloatTopic("Subsystems/Launcher/Launcher_Amps").publish()
            self._at_speed = self.nt_instance.getBooleanTopic("Subsystems/Launcher/At_Speed").publish()

        self._leftMotor = rev.SparkMax(
            self.Constants.LeftMotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._rightMotor = rev.SparkMax(
            self.Constants.RightMotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._leftMotor.setCANTimeout(250)
        self._rightMotor.setCANTimeout(250)
        self._controller = self._leftMotor.getClosedLoopController()
        spark_config = rev.SparkMaxConfig()
        spark_config.inverted(True)
        spark_config.smartCurrentLimit(self.Constants.MotorCurrentLimit)
        spark_config.voltageCompensation(self.Constants.MotorVComp)
        spark_config.closedLoop \
            .setFeedbackSensor(rev.FeedbackSensor.kPrimaryEncoder) \
            .pidf(0.33, 0.0, 0.0, 0.3)
        # 1:1000 1k RPM
        # 1:1.5 gear reduction
        gear_reduction = 1.0 / 1000.0 / 1.5
        spark_config.encoder \
            .positionConversionFactor(gear_reduction) \
            .velocityConversionFactor(gear_reduction)
        self._leftMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        spark_config.follow(self.Constants.LeftMotorId, True)
        self._rightMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        self._encoder = self._leftMotor.getEncoder()
        if not RobotBase.isReal():
            from wpilib.simulation import FlywheelSim
            from wpimath.system.plant import DCMotor, LinearSystemId
            from rev import SparkMaxSim

            kMoI = units.kilogram_square_meters(0.005)  # kg*m^2, just a guess
            kGearing = 40.0 / 60.0  # 40:60 gearing on the launcher
            motor = DCMotor.NEO(2)
            system = LinearSystemId.flywheelSystem(motor, kMoI, kGearing)
            self._flywheelSim = FlywheelSim(system, motor, [0.01])
            self._motorSim = SparkMaxSim(self._leftMotor, DCMotor.NEO(2))

    def periodic(self) -> None:
        """Updates the current speed of the launcher on the dashboard."""
        dist = self._tracker.target_dist
        self._speed_entry.set(self.SPEED_LOOKUP_SLOPE * dist + self.SPEED_LOOKUP_INTERCEPT)
        if ENABLE_TELEMETRY:
            self._cur_speed.set(self._encoder.getVelocity())
            self._at_speed.set(self.at_speed())
            self._cur_amps.set(self._leftMotor.getOutputCurrent())

    def simulationPeriodic(self) -> None:
        vbus = RobotController.getBatteryVoltage()
        dt = 0.02  # Assuming a fixed 20ms simulation loop
        voltage = vbus * self._motorSim.getAppliedOutput()
        self._flywheelSim.setInputVoltage(voltage)
        self._flywheelSim.update(dt)
        velocity = self._flywheelSim.getAngularVelocity()
        velocity_rpm = units.radiansPerSecondToRotationsPerMinute(velocity)
        velocity_final = velocity_rpm
        self._motorSim.iterate(
            velocity_final,
            vbus,
            dt
        )

    def start(self) -> Command:
        """Starts the launcher at launch speed."""

        def command_function():
            self._controller.setSetpoint(self._speed_entry.get(), rev.SparkLowLevel.ControlType.kVelocity)

        return self.run(command_function).withName("LauncherStart")

    def stop(self) -> Command:
        """Stops the launcher."""

        def command_function():
            self._leftMotor.set(0)

        return self.run(command_function).withName("LauncherStop")

    def at_speed(self) -> bool:
        """Returns True if the launcher is at launch speed."""
        v = self._encoder.getVelocity()
        return abs(v) >= self._speed_entry.get() * 0.9
