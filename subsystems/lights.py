#!/usr/bin/env python3.10
import dataclasses
from wpilib import SerialPort, Timer
from commands2 import Subsystem


@dataclasses.dataclass
class LightPattern:
    NO_SIGNAL = 0        # FLASH ORANGE/YELLOW
    DS_DISCONNECTED = 1  # FLASH YELLOW/OFF
    ERROR = 2            # RAPID RED FLASH
    IDLE = 3             # SOLID BLUE
    SHOWTIME = 4         # SOMETHING COOL
    READY_TO_SHOOT = 5   # SOLID GREEN
    SHOOTING = 6         # GREEN, FLASHES OR CHASING STRIPES OF WHITE

_BUFFERS = tuple(b"P=%d\n" % i for i in range(7))
_KEEP_ALIVE_INTERVAL = 1.0  # seconds

class LEDSubsystem(Subsystem):
    def __init__(self):
        self.serialPort = SerialPort(115200, SerialPort.Port.kUSB)
        self._next_ping = 0

    def set_pattern(self, pattern: int):
        buffer = _BUFFERS[pattern] if 0 <= pattern <= 6 else b"\n"
        self._send(buffer)
    
    def periodic(self):
        current_time = Timer.getFPGATimestamp()
        if self._next_ping <= current_time:
            self._send(b"\n")
    
    def _send(self, buffer):
        self.serialPort.write(buffer)
        self._next_ping = Timer.getFPGATimestamp() + _KEEP_ALIVE_INTERVAL