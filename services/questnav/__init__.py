from wpilib import RobotBase
from services.questnav.questnav_data import PoseFrame

if RobotBase.isReal():
    from services.questnav.questnav import QuestNav
else:
    from services.questnav.questnav_stub import QuestNavStub as QuestNav