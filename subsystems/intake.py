import rev
from commands2 import Subsystem, Command
import ntcore
from wpilib import RobotBase, RobotController
from wpimath import units

import constants

class IntakeExtender(Subsystem):
    Constants = constants.Subsystems.IntakeExtender
    _targetPosition: float = 0
    
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        topicKey = "Subsystems/Intake"
        self._reverse_limit_entry = self.nt_instance.getFloatTopic(f"{topicKey}/Extender_ReverseLimit").getEntry(self.Constants.ReverseLimit)
        self._forward_limit_entry = self.nt_instance.getFloatTopic(f"{topicKey}/Extender_ForwardLimit").getEntry(self.Constants.ForwardLimit)
        self._reverse_limit_entry.setDefault(self.Constants.ReverseLimit)
        self._forward_limit_entry.setDefault(self.Constants.ForwardLimit)

        self._leftMotor = rev.SparkMax(self.Constants.LeftMotorId,rev.SparkBase.MotorType.kBrushless)
        self._rightMotor = rev.SparkMax(self.Constants.RightMotorId,rev.SparkBase.MotorType.kBrushless)
    
        # Create motor config
        spark_config = rev.SparkMaxConfig()
        spark_config.inverted(True)
        spark_config.smartCurrentLimit(self.Constants.MotorCurrentLimit)
        spark_config.closedLoop \
            .pid(*self.Constants.PID)
        spark_config.closedLoop.maxMotion \
            .cruiseVelocity(self.Constants.MotorSpeed) \
            .maxAcceleration(self.Constants.MaxAcceleration) \
            .allowedProfileError(self.Constants.AllowedProfileError)
        spark_config.encoder \
            .positionConversionFactor(self.Constants.GearReduction) \
            .velocityConversionFactor(self.Constants.GearReduction / 60.0)
        self._closedLoopController = self._leftMotor.getClosedLoopController()
        self.encoder = self._leftMotor.getEncoder()
        self.encoder.setPosition(0)
        spark_config.voltageCompensation(self.Constants.MotorVComp)
        # Configure Leader
        self._leftMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        # Configure Follower
        spark_config.follow(self.Constants.LeftMotorId,True)
        self._rightMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
        self.encoder.setPosition(0)

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
        self._targetPosition = self._forward_limit_entry.get()

        def command_function():
            # self._closedLoopController.setReference(self._targetPosition, rev.SparkBase.ControlType.kMAXMotionPositionControl)
            self._leftMotor.set(0.0)

        return self.run(command_function).withName("IntakeStop")
    
    def extend(self) -> Command:
        """Extends the intake mechanism."""
        self._targetPosition = self._forward_limit_entry.get()

        def command_function():
            # self._closedLoopController.setReference(self._targetPosition, rev.SparkBase.ControlType.kMAXMotionPositionControl)
            self._leftMotor.set(0.3)

        return self.run(command_function).withName("IntakeExtend")

    def retract(self) -> Command:
        """Retracts the intake mechanism."""
        self._targetPosition = self._reverse_limit_entry.get()

        def command_function():
            # self._closedLoopController.setReference(self._targetPosition, rev.SparkBase.ControlType.kMAXMotionPositionControl)
            self._leftMotor.set(-0.3)

        return self.run(command_function).withName("IntakeRetract")

class Intake(Subsystem):
    """Controls the intake mechanism for collecting game pieces."""

    Constants = constants.Subsystems.Intake

    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic("Subsystems/Intake/Intake_Speed").getEntry(self.Constants.IntakeSpeed)
        self._speed_entry.setDefault(self.Constants.IntakeSpeed)
        self._motor = rev.SparkMax(
            self.Constants.MotorId,
            rev.SparkBase.MotorType.kBrushless
        )
        self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        self._motor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

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

    
