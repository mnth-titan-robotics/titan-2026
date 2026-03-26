from .led_pattern import LEDPattern
from wpilib import AddressableLED


ColorOrder = AddressableLED.ColorOrder
LEDData = AddressableLED.LEDData


class EmbossPattern(LEDPattern):
    def __init__(self, steps_per_second: float):
        super().__init__(steps_per_second)
        self._head = 0

    def step(self, led_strip_array: list[LEDData]) -> None:
        current_led = led_strip_array[self._head]
        
        if current_led.r == 0:
            led_strip_array[self._head].setRGB(255, 255, 255)

        else:
            led_strip_array[self._head].setRGB(0, 0, 0)

        if self._head == (len(led_strip_array) - 1):
            self._head = 0
        
        else:
            self._head += 1