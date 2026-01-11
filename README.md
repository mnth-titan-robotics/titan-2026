# Introduction
This project provides a starting point for coding an FRC robot.

## Glossary
* **SubSystem**: A component of the robot (eg: Drive, Launcher, Roller). All motor controllers, pneumatic controllers, lights and other outputs should be "owned" by a single subsystem and controlled only through **Commands** (see below). 
* **Command**: The way we interact with SubSystems. Each SubSystem can run a single command at a time - the Drive SubSystem can either drive or stop, but not both at the same time.
* **Sensor**: Reads data from a physical sensor (light, color, distance). 
* **RobotContainer**: Responsible for creating all subsystems, sensors, commands and setting up the controls for the robot.

## Project Structure
* **commands**
  * **game.py**: Combines subsystem commands into sequences of complex functionality. For example, we might drive to a target location while moving an elevator up, then activate the roller when we're close enough.
  * **auto.py**: A collection of commands used during Autonomous operation. These
* **subsystems**: Contains robot subsystems - typically one-per-file.
* **README.md**: This file
* **robot.py**: The entry point for our robot. Creates the robot container, any code that should run when starting auto, starting teleop, disabling, etc.
* **robotcontainer.py**: Contains RobotContainer (see **Glossary** above)

# Setup
## Prerequisites
* [Visual Studio Code](https://code.visualstudio.com/download) is installed

## Getting Started
1. Clone the repository
   1. Open the Command Palette (Ctrl-Shift-P on Windows, Command-Shift-P on Mac)
   2. Select `Git: Clone`
   3. Paste `https://github.com/mnth-titan-robotics/robotpy-base.git`
   4. Press Enter
   5. Choose a folder where you'd like to store your code (eg: c:\frc\ or Documents\frc)
   6. Click `Select as Repository Destination`
2. Create a python virtual environment
   1. Open the Command Palette
   2. Select `Python: Create Environment...`
   3. Select `Quick Create`
3. Install prerequisites
   1. Open the Command Palette
   2. Select `Tasks: Run Task`
   3. Run `RobotPy Project: Install Prerequisites`
   4. Wait for the Terminal to show `Terminal will be reused by tasks, press any key to close it.`
4. Install robotpy libraries
   1. Open the Command Palette
   2. Select `Tasks: Run Task`
   3. Run `RobotPy Project: Install Prerequisites`