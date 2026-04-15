import ntcore
from wpilib import SmartDashboard, SendableChooser


LEDPatterns = {
    "Emboss Pattern": 0,
    "Chaser Pattern": 1,
    "White Fading Pattern": 2
}


class LEDSubsystem:
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.led_patterns = SendableChooser()

        self.create_patterns()
        
        SmartDashboard.putData("Subsystems/Lights2/LEDPatterns", self.led_patterns)
        self.led_patterns.onChange(self.send_pattern)

    def create_patterns(self):
        for pattern_name, pattern_value in LEDPatterns.items():
            self.led_patterns.addOption(pattern_name, pattern_value)

    def send_pattern(self, args):
        print(args)
