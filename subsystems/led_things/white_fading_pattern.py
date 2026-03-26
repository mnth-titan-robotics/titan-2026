from .led_pattern import LEDPattern
from wpilib import AddressableLED
import math


ColorOrder = AddressableLED.ColorOrder
LEDData = AddressableLED.LEDData


class WhiteFadingPattern(LEDPattern):
    def __init__(self, steps_per_second: float):
        super().__init__(steps_per_second)
        self._intensity = 0
        self._fading_out = False

    def _changeIntensity(self):
        fading_value = 1.05
        if not self._fading_out:
            if self._intensity == 0:
                self._intensity += 5
            
            else:
                future_value = self._intensity * fading_value

                if future_value > 255:
                    self._intensity = 255
                    self._fading_out = True
                
                else:
                    self._intensity = future_value
        
        else:
            future_value = self._intensity / fading_value

            if future_value < 1:
                self._intensity = 0
                self._fading_out = False
            
            else:
                self._intensity = future_value            

    def step(self, led_strip_array: list[LEDData]) -> None:
        print(self._intensity)

        for list_obj in led_strip_array:
            list_obj.setRGB(round(self._intensity), round(self._intensity), round(self._intensity))

        self._changeIntensity()