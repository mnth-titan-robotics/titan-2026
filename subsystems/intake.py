import rev
from commands2 import Subsystem, Command, cmd
import ntcore
from wpilib import RobotBase, RobotController
from wpimath import units
from configs import Configs

import constants

ENABLE_TELEMETRY = constants.ENABLE_TELEMETRY

class IntakeExtender(Subsystem):
    Constants = constants.Subsystems.IntakeExtender
    _targetPosition: float = 0

    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        topicKey = "Subsystems/Intake"
        self._reverse_limit_entry = self.nt_instance.getFloatTopic(
            f"{topicKey}/Extender_ReverseLimit").getEntry(self.Constants.ReverseLimit)
        self._forward_limit_entry = self.nt_instance.getFloatTopic(
            f"{topicKey}/Extender_ForwardLimit").getEntry(self.Constants.ForwardLimit)
        self._pos_publisher = self.nt_instance.getFloatTopic(f"{topicKey}/Extender_Pos").publish()
        self._reverse_limit_entry.setDefault(self.Constants.ReverseLimit)
        self._forward_limit_entry.setDefault(self.Constants.ForwardLimit)

        self._leftMotor = rev.SparkMax(self.Constants.LeftMotorId, rev.SparkBase.MotorType.kBrushless)
        self._rightMotor = rev.SparkMax(self.Constants.RightMotorId, rev.SparkBase.MotorType.kBrushless)

        # Create motor config
        self._controller = self._leftMotor.getClosedLoopController()
        # Configure Leader
        self._leftMotor.configure(
            Configs.IntakeExtender.kLeftConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters)

        # Configure Follower
        self._rightMotor.configure(
            Configs.IntakeExtender.kRightConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters)
        
        self.encoder = self._leftMotor.getEncoder()
        self.encoder.setPosition(0)
        if ENABLE_TELEMETRY:
            self._cur_speed = self.nt_instance.getFloatTopic("Subsystems/Launcher/Launcher_Velocity").publish()
            self._cur_amps = self.nt_instance.getFloatTopic("Subsystems/Launcher/Launcher_Amps").publish()
            self._at_speed = self.nt_instance.getBooleanTopic("Subsystems/Launcher/At_Speed").publish()

        if not RobotBase.isReal():
            from wpilib.simulation import SingleJointedArmSim
            from wpimath.system.plant import DCMotor, LinearSystemId

            kMoI = units.kilogram_square_meters(0.005)  # kg*m^2, just a guess
            kGearing = 3.0 * 4.0 * 5.0 * 64.0 / 26.0
            kArmLength = units.inchesToMeters(14)
            kMinAngle = units.radians(0)
            kMaxAngle = units.radians(95.0)
            motor = DCMotor.NEO(2)
            system = LinearSystemId.singleJointedArmSystem(motor, kMoI, kGearing)
            self._armSim = SingleJointedArmSim(
                system,
                motor,
                kGearing,
                kArmLength,
                kMinAngle,
                kMaxAngle,
                True,
                kMinAngle
            )
            self._motorSim = rev.SparkMaxSim(self._leftMotor, DCMotor.NEO(2))

    def periodic(self) -> None:
        """Updates the current state of the intake."""
        self._pos_publisher.set(self.encoder.getPosition())
        if ENABLE_TELEMETRY:
            self._cur_speed.set(self.encoder.getVelocity())
            self._cur_amps.set(self._leftMotor.getOutputCurrent())

    def simulationPeriodic(self) -> None:
        vbus = RobotController.getBatteryVoltage()
        dt = 0.02  # Assuming a fixed 20ms simulation loop
        voltage = vbus * self._motorSim.getAppliedOutput()
        self._armSim.setInputVoltage(voltage)
        self._armSim.update(dt)
        velocity = self._armSim.getVelocity()
        velocity_rpm = units.radiansPerSecondToRotationsPerMinute(velocity)
        velocity_final = velocity_rpm
        self._motorSim.iterate(
            velocity_final,
            vbus,
            dt
        )

    def resetEncoder(self) -> None:
        self.encoder.setPosition(0)

    def stop(self) -> Command:
        def command_function():
            self._leftMotor.set(0.0)

        return self.run(command_function).withName("IntakeStop")

    def extend(self) -> Command:
        """Extends the intake mechanism."""
        def command_function():
            self._leftMotor.set(0.3)

        return self.run(command_function).withName("IntakeExtend")

    def retract(self) -> Command:
        """Retracts the intake mechanism."""
        def command_function():
            self._leftMotor.set(-0.3)

        return self.run(command_function).withName("IntakeRetract")

    def auto_extend(self) -> Command:
        """Extends the intake mechanism."""
        def command_function():
            self._controller.setSetpoint(self.Constants.ExtendPosition, rev.SparkLowLevel.ControlType.kPosition)

        return self.run(command_function) \
            .until(self.is_extended) \
            .withName("IntakeAutoExtend") \
            .withTimeout(1.0)

    def auto_retract(self) -> Command:
        """Retracts the intake mechanism."""
        def command_function():
            self._controller.setSetpoint(self.Constants.RetractPosition, rev.SparkLowLevel.ControlType.kPosition)

        return self.run(command_function) \
            .until(self.is_retracted) \
            .withName("IntakeAutoRetract")

    def is_retracted(self) -> bool:
        return self.encoder.getPosition() < self.Constants.RetractPosition + self.Constants.AllowedProfileError
    
    def is_extended(self) -> bool:
        return self.encoder.getPosition() > self.Constants.ExtendPosition - self.Constants.AllowedProfileError


class Intake(Subsystem):
    """Controls the intake mechanism for collecting game pieces."""

    Constants = constants.Subsystems.Intake

    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic(
            "Subsystems/Intake/Intake_Speed").getEntry(self.Constants.IntakeSpeed)
        self._speed_entry.setDefault(self.Constants.IntakeSpeed)
        self._motor = rev.SparkMax(
            self.Constants.MotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._motor.setCANTimeout(250)
        self._motor.configure(
            Configs.Intake.kConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters)

        if not RobotBase.isReal():
            from wpilib.simulation import FlywheelSim
            from wpimath.system.plant import DCMotor, LinearSystemId

            kMoI = units.kilogram_square_meters(0.005)  # kg*m^2, just a guess
            kGearing = 5.0 * 4.0
            motor = DCMotor.NEO(2)
            system = LinearSystemId.flywheelSystem(motor, kMoI, kGearing)
            self._flywheelSim = FlywheelSim(system, motor)
            self._motorSim = rev.SparkMaxSim(self._motor, DCMotor.NEO(1))

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

    def intake(self) -> Command:
        """Starts the intake at intake speed."""
        def command_function():
            self._motor.set(self._speed_entry.get())

        return self.run(command_function).withName("IntakeStart")
    
    def intake_half_speed(self) -> Command:
        """Starts the intake at half intake speed"""
        def command_function():
            motor_speed = self._speed_entry.get()
            self._motor.set((motor_speed/2))
            
        return self.run(command_function).withName("IntakeStartAtHalfSpeed")

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