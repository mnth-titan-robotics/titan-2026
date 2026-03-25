from .led_pattern import LEDPattern
from wpilib import AddressableLED


ColorOrder = AddressableLED.ColorOrder
LEDData = AddressableLED.LEDData


class ChaserPattern(LEDPattern):
    def __init__(self, steps_per_second: float, length: int):
        super().__init__(steps_per_second)
        self._head = 0
        self._length = length

    def step(self, led_strip_array: list[LEDData]) -> None:
        chaser_positions = []

        for i in range(self._length):
            if i == 0:
                chaser_positions.append(0)
            else:
                chaser_positions.append(-i)
        
        for i in range(len(chaser_positions) + 1):
            if i < len(chaser_positions):
                chaser_position = (self._head + chaser_positions[i])
                led_strip_array[chaser_position].setRGB(255, 255, 255)
                
            else:
                chaser_position = (self._head + (chaser_positions[-1] - 1))
                led_strip_array[chaser_position].setRGB(0, 0, 0)
        
        if self._head == (len(led_strip_array) - 1):
            self._head = 0
        
        else:
            self._head += 1