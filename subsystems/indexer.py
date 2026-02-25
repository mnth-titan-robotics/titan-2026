import rev
from commands2 import Subsystem, Command
import ntcore
import constants

class Indexer(Subsystem):
    Constants = constants.Subsystems.Indexer
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic("Indexer/Speed").getEntry(self.Constants.IndexerSpeed)
        self._motor = rev.SparkMax(self.Constants.MotorHighId,rev.SparkBase.MotorType.kBrushless)

    def stop(self) -> Command:
        """Stops all motors"""
        def command_function():
            # Placeholder for retraction logic
            self._motor.set(0)

        return self.run(command_function).withName("IndexerStop")
    
    def feed(self) -> Command:
        """Turns on the low motors"""
        def command_function():
            # Placeholder for retraction logic
            self._motor.set(self._speed_entry.get())

        return self.run(command_function).withName("IndexerFeed")
   
    def reverse(self) -> Command:
        """Turns on all motors"""
        def command_function():
            # Placeholder for retraction logic
            self._motor.set(-self._speed_entry.get())

        return self.run(command_function).withName("IndexerReverse")
