from wpilib import AddressableLED, RobotBase
from commands2 import Subsystem


ColorOrder = AddressableLED.ColorOrder
LEDData = AddressableLED.LEDData


class LEDSubsystem(Subsystem):
    tickCounter = 1
    MAXIMUM_TICK_RATE = 50

    def __init__(self):
        # Hardcoded Variables
        LED_PORT_0 = 0

        # Class-wide Variables
        self.ACTIVE_LED = 0

        self.LED_STRIP_LENGTH = 35
        self.LED_OBJECT = AddressableLED(LED_PORT_0)
        self.LED_DATA_LIST = list(LEDData() for i in range(self.LED_STRIP_LENGTH))

        # "startup" functions
        self._setupLEDs()

        self.LED_OBJECT.start()
        if RobotBase.isSimulation():
            from wpilib.simulation import AddressableLEDSim
            self._sim_led_object = AddressableLEDSim(self.LED_OBJECT)

    def _setupLEDs(self):
        self.LED_OBJECT.setLength(self.LED_STRIP_LENGTH)
        self.LED_OBJECT.setColorOrder(ColorOrder.kRGB)

    def _incrementTickCounter(self):
        if self.tickCounter != self.MAXIMUM_TICK_RATE:
            self.tickCounter += 1

        else:
            self.tickCounter = 1

    def _incrementActiveLED(self):
        if self.ACTIVE_LED < (self.LED_STRIP_LENGTH - 1):
            self.ACTIVE_LED += 1
        
        else:
            self.ACTIVE_LED = 0

    def LEDPattern_Chaser(self, CHASER_WIDTH=1):
        chaser_positions = []

        for i in range(CHASER_WIDTH):
            if i == 0:
                chaser_positions.append(0)
            else:
                chaser_positions.append(-i)
        
        for i in range(len(chaser_positions) + 1):
            if i < len(chaser_positions):
                chaser_position = (self.ACTIVE_LED + chaser_positions[i])
                self.LED_DATA_LIST[chaser_position].setRGB(255, 255, 255)
                
            else:
                chaser_position = (self.ACTIVE_LED + (chaser_positions[-1] - 1))
                self.LED_DATA_LIST[chaser_position].setRGB(0, 0, 0)

    def LEDPattern_Emboss(self):
        led = self.LED_DATA_LIST[self.ACTIVE_LED]
        led_colors = led.r, led.g, led.b

        if led_colors[0] == 0:
            self.LED_DATA_LIST[self.ACTIVE_LED].setRGB(255, 255, 255)

        else:
            self.LED_DATA_LIST[self.ACTIVE_LED].setRGB(0, 0, 0)

    def LEDPattern_Rainbow(self):
        pass

    def periodic(self):
        self._incrementTickCounter()

        #self.LEDPattern_Chaser(CHASER_WIDTH=8)
        self.LEDPattern_Emboss()
        #self.LEDPattern_Rainbow()

        self._incrementActiveLED()
        self.LED_OBJECT.setData(self.LED_DATA_LIST)