import rev
from commands2 import Subsystem, Command
import ntcore
import constants
from configs import Configs

class Indexer(Subsystem):
    Constants = constants.Subsystems.Indexer
    def __init__(self):
        self._feeding = False
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._speed_entry = self.nt_instance.getFloatTopic("Subsystems/Indexer/Indexer_Speed").getEntry(self.Constants.IndexerSpeed)
        self._speed_entry.setDefault(self.Constants.IndexerSpeed)
        self._motor = rev.SparkMax(self.Constants.MotorId,rev.SparkBase.MotorType.kBrushless)
        self._motor.configure(
            Configs.Indexer.kConfig, 
            rev.ResetMode.kResetSafeParameters, 
            rev.PersistMode.kPersistParameters)

    def stop(self) -> Command:
        """Stops all motors"""
        def command_function():
            # Placeholder for retraction logic
            self._motor.set(0)
            self._feeding = False

        return self.run(command_function).withName("IndexerStop")
    
    def feed(self) -> Command:
        """Turns on the low motors"""
        def command_function():
            # Placeholder for retraction logic
            self._motor.set(self._speed_entry.get())
            self._feeding = True

        return self.run(command_function).withName("IndexerFeed")
   
    def reverse(self) -> Command:
        """Turns on all motors"""
        def command_function():
            # Placeholder for retraction logic
            self._motor.set(-self._speed_entry.get())
            self._feeding = False

        return self.run(command_function).withName("IndexerReverse")
    
    def is_feeding(self) -> bool:
        """Returns true if the indexer is running"""
        return self._feeding
