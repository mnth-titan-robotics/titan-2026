import traceback
from commands2 import CommandScheduler, Command, cmd
from wpilib import DataLogManager, DriverStation, Timer, SmartDashboard


def start() -> None:
    DataLogManager.start()
    DriverStation.startDataLog(DataLogManager.getLog())

    CommandScheduler.getInstance().onCommandInitialize(
        lambda command: log(f'----> Command Start: {command.getName()}')
    )
    CommandScheduler.getInstance().onCommandInterrupt(
        lambda command: log(f'----X Command Interrupt: {command.getName()}')
    )
    CommandScheduler.getInstance().onCommandFinish(
        lambda command: log(f'----< Command End: {command.getName()}')
    )

    SmartDashboard.putBoolean("Robot/Status/HasError", False)
    SmartDashboard.putString("Robot/Status/LastError", "")

    log("********** Robot Started **********")


def log(message: str) -> None:
    DataLogManager.log(f'[{"%.6f" % Timer.getFPGATimestamp()}] {message}')


def log_(message: str) -> Command:
    return cmd.runOnce(lambda: log(message))


def debug(message: str) -> None:
    log(f'@@@@@@@@@@ DEBUG: {message} @@@@@@@@@@')


def debug_(message: str) -> Command:
    return cmd.runOnce(lambda: debug(message))


def error(message: str) -> None:
    log(f'!!!!!!!!!! ERROR: {message} !!!!!!!!!!')
    SmartDashboard.putBoolean("Robot/Status/HasError", True)
    SmartDashboard.putString("Robot/Status/LastError", message)


def exception() -> None:
    error(traceback.format_exc())
