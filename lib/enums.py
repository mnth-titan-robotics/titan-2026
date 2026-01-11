from enum import Enum, IntEnum, auto

class Alliance(IntEnum):
  Red = 0
  Blue = 1

class RobotMode(Enum):
  Disabled = auto()
  Auto = auto()
  Teleop = auto()
  Test = auto()

class RobotState(Enum):
  Disabled = auto()
  Enabled = auto()
  EStopped = auto()

class OptionState(Enum):
  Enabled = auto()
  Disabled = auto()

class MotorDirection(Enum):
  Forward = auto()
  Reverse = auto()
  Stopped = auto()

class MotorIdleMode(Enum):
  Brake = auto()
  Coast = auto()

class MotorControllerType(Enum):
  SparkMax = auto()
  SparkFlex = auto()

class DriveOrientation(Enum):
  Field = auto()
  Robot = auto()

class SpeedMode(Enum):
  Competition = auto()
  Demo = auto()

class LockState(Enum):
  Unlocked = auto()
  Locked = auto()

class TargetAlignmentMode(Enum):
  Heading = auto()
  Translation = auto()

class ControllerRumbleMode(Enum):
  Both = auto()
  Driver = auto()
  Operator = auto()

class ControllerRumblePattern(Enum):
  Short = auto()
  Long = auto()

class ModuleLocation(IntEnum):
  LeftFront = auto()
  LeftRear = auto()
  RightFront = auto()
  RightRear = auto()