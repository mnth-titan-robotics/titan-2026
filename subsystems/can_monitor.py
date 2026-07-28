from wpilib import RobotController
from commands2 import Subsystem


class CANHealthMonitor(Subsystem):
    FAULT_THRESHOLD = 3   # new faults in one frame to trip
    CLEAR_UPDATES = 5     # consecutive clean frames to reset

    def __init__(self):
        self._is_can_faulted = False
        self._clean_frames = 0
        self._last_fault_count = self._total_faults()

    def _total_faults(self) -> int:
        s = RobotController.getCANStatus()
        return (
            s.busOffCount
            + s.txFullCount
            + s.receiveErrorCount
            + s.transmitErrorCount
        )

    @property
    def is_can_faulted(self) -> bool:
        return self._is_can_faulted

    def periodic(self):
        current = self._total_faults()
        new_faults = current - self._last_fault_count
        self._last_fault_count = current

        if new_faults > self.FAULT_THRESHOLD:
            self._is_can_faulted = True
            self._clean_frames = 0
        elif new_faults == 0:
            self._clean_frames += 1
            if self._clean_frames >= self.CLEAR_UPDATES:
                self._is_can_faulted = False
        else:
            # 1–3 new faults: not enough to trip, but breaks the clean streak
            self._clean_frames = 0