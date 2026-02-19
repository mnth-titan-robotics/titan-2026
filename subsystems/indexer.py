import rev
from commands2 import Subsystem, Command

import constants

class Indexer(Subsystem):
    Constants = constants.Subsystems.Indexer
    def __init__(self):
        self.MotorHigh = rev.SparkMax(self.Constants.MotorHighId,rev.SparkBase.MotorType.kBrushless)
        self.MotorLow = rev.SparkMax(self.Constants.MotorLowId,rev.SparkBase.MotorType.kBrushless)
   
    def stop(self) -> Command:
        """Stops all motors"""
        def command_function():
            # Placeholder for retraction logic
            pass

        return self.run(command_function).withName("IndexerStop")
    
    def feed(self) -> Command:
        """Turns on the low motors"""
        def command_function():
            # Placeholder for retraction logic
            pass

        return self.run(command_function).withName("IndexerFeed")
   
    def shoot(self) -> Command:
        """Turns on all motors"""
        def command_function():
            # Placeholder for retraction logic
            pass

        return self.run(command_function).withName("IndexerShoot")
