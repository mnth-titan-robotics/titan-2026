from abc import ABC, abstractmethod
from wpilib import AddressableLED
import math


ColorOrder = AddressableLED.ColorOrder
LEDData = AddressableLED.LEDData


class LEDPattern(ABC):
    def __init__(self, steps_per_second: float):
        self._ticks_per_step = round(50.0 / steps_per_second)
        self._tick_count = 0

    def tick(self, led_strip_array: list[LEDData]) -> None:
        self._tick_count += 1
        if self._tick_count >= self._ticks_per_step:
            self._tick_count = 0
            self.step(led_strip_array)

    @abstractmethod
    def step(self, led_strip_array: list[LEDData]) -> None:
        pass