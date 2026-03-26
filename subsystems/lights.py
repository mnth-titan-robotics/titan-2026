from wpilib import AddressableLED, RobotBase, Color
from commands2 import Subsystem

from .led_things.led_pattern import LEDPattern
from .led_things.chaser_pattern import ChaserPattern
from .led_things.emboss_pattern import EmbossPattern
from .led_things.white_fading_pattern import WhiteFadingPattern


ColorOrder = AddressableLED.ColorOrder
LEDData = AddressableLED.LEDData


class LEDSubsystem(Subsystem):
    tickCounter = 1
    MAXIMUM_TICK_RATE = 50

    def __init__(self):
        # Hardcoded Variables
        LED_PORT_0 = 0

        # Class-wide Variables
        self._LED_STRIP_LENGTH = 35
        self._LED_OBJECT = AddressableLED(LED_PORT_0)
        self._LED_DATA_LIST = list(LEDData() for i in range(self._LED_STRIP_LENGTH))
        
        #self._pattern: LEDPattern = ChaserPattern(steps_per_second=50, length=5)
        #self._pattern: LEDPattern = EmbossPattern(steps_per_second=25)
        self._pattern: LEDPattern = WhiteFadingPattern(steps_per_second=50)

        # "startup" functions
        self._setupLEDs()

        self._LED_OBJECT.start()
        if RobotBase.isSimulation():
            from wpilib.simulation import AddressableLEDSim
            self._sim_led_object = AddressableLEDSim(self._LED_OBJECT)

    def _setupLEDs(self):
        self._LED_OBJECT.setLength(self._LED_STRIP_LENGTH)
        self._LED_OBJECT.setColorOrder(ColorOrder.kRGB)

    def _clearLEDs(self):
        for list_obj in self._LED_DATA_LIST:
            list_obj.setRGB(0, 0, 0)

    def set_pattern(self, pattern: LEDPattern) -> None:
        self._clearLEDs()
        self._pattern = pattern

    # def LEDPattern_Rainbow(self):
    #     pass

    def periodic(self):
        self._pattern.tick(self._LED_DATA_LIST)
        self._LED_OBJECT.setData(self._LED_DATA_LIST)