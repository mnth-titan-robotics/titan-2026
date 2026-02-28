import rev
from commands2 import Subsystem, Command
import ntcore

import constants

class IntakeExtender(Subsystem):
    Constants = constants.Subsystems.IntakeExtender
    _targetPosition: float = 0
    
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._reverse_limit_entry = self.nt_instance.getFloatTopic("IntakeExtender/ReverseLimit").getEntry(self.Constants.ReverseLimit)
        self._forward_limit_entry = self.nt_instance.getFloatTopic("IntakeExtender/ForwardLimit").getEntry(self.Constants.ForwardLimit)

        self._leftMotor = rev.SparkMax(self.Constants.LeftMotorId,rev.SparkBase.MotorType.kBrushless)
        self._rightMotor = rev.SparkMax(self.Constants.RightMotorId,rev.SparkBase.MotorType.kBrushless)
    
        # Create motor config
        spark_config = rev.SparkMaxConfig()
        spark_config.inverted(False)
        spark_config.smartCurrentLimit(self.Constants.MotorCurrentLimit)
        spark_config.closedLoop \
            .pid(*self.Constants.PID)
        spark_config.closedLoop.maxMotion \
            .cruiseVelocity(self.Constants.MotorSpeed) \
            .maxAcceleration(self.Constants.MaxAcceleration) \
            .allowedProfileError(self.Constants.AllowedProfileError)
        self._closedLoopController = self._leftMotor.getClosedLoopController()
        self._relativeEncoder = self._leftMotor.getEncoder()
        self._relativeEncoder.setPosition(0)
        spark_config.voltageCompensation(self.Constants.MotorVComp)
        # Configure Leader
        self._leftMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

        # Configure Follower
        spark_config.follow(self.Constants.LeftMotorId,True)
        self._rightMotor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)
    
    def extend(self) -> Command:
        """Extends the intake mechanism."""
        self._targetPosition = self._forward_limit_entry.get()

        def command_function():
            self._closedLoopController.setReference(self._targetPosition, rev.SparkBase.ControlType.kMAXMotionPositionControl)

        return self.runOnce(command_function).withName("IntakeExtend")

    def retract(self) -> Command:
        """Retracts the intake mechanism."""
        self._targetPosition = self._reverse_limit_entry.get()

        def command_function():
            self._closedLoopController.setReference(self._targetPosition, rev.SparkBase.ControlType.kMAXMotionPositionControl)

        return self.run(command_function).withName("IntakeRetract")

class Intake(Subsystem):
    """Controls the intake mechanism for collecting game pieces."""

    Constants = constants.Subsystems.Intake

    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic("Intake/Speed").getEntry(self.Constants.IntakeSpeed)
        self._motor = rev.SparkMax(
            self.Constants.MotorId,
            rev.SparkBase.MotorType.kBrushed
        )
        # self._motor.setCANTimeout(250)
        spark_config = rev.SparkMaxConfig()
        self._motor.configure(spark_config, rev.ResetMode.kResetSafeParameters, rev.PersistMode.kPersistParameters)

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

    
