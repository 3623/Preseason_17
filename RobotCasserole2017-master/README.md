![logo](http://robotcasserole.org/wp-content/uploads/2017/01/banner_2017_text.png)

# RobotCasserole2017
Robot Casserole robot source code for the 2017 FRC game, First Steamworks.

## Contents

Here's a high level overview of what we did this year:
- [Path-planned autonomous on a mechanum drivetrain](https://github.com/RobotCasserole1736/MecanumPathPlanner)
- [High goal vision target detection](https://github.com/RobotCasserole1736/RobotCasserole2017/wiki/Vision-Target-Qualification-Algorithm)
- [Singelton](https://github.com/RobotCasserole1736/RobotCasserole2017/wiki/Singelton-Architecture) implementation of key robot components
- Custom robot [website](https://github.com/RobotCasserole1736/RobotCasserole2017/tree/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/lib/WebServer) for real-time driver information, debugging information, and value calibration
- PID control of [drivetrain velocity](https://github.com/RobotCasserole1736/RobotCasserole2017/blob/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/robot/DriveTrain.java), [vision alignment](https://github.com/RobotCasserole1736/RobotCasserole2017/blob/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/robot/VisionAlignment.java), and [shooter wheel](https://github.com/RobotCasserole1736/RobotCasserole2017/blob/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/robot/ShooterWheelCtrl.java)
- Automated ball launch [system](https://github.com/RobotCasserole1736/RobotCasserole2017/blob/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/robot/ShotControl.java) with integrated control of shooter, vision, and hopper feed
- Advanced robot performance monitoring and logging

### Robot Code

Eclipse project containing [2017-specific code](https://github.com/RobotCasserole1736/RobotCasserole2017/tree/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/robot), Casserole common [library code](https://github.com/RobotCasserole1736/RobotCasserole2017/tree/master/EclipseProject/RobotCasserole2017/src/org/usfirst/frc/team1736/lib), external jar libraries, and [robot website resources](https://github.com/RobotCasserole1736/RobotCasserole2017/tree/master/EclipseProject/RobotCasserole2017/resources).

### BBB

[Python source code](https://github.com/RobotCasserole1736/RobotCasserole2017/blob/master/BBB/main.py) for our vision target identification algorithm running on a Beaglebone Black coprocessor. More information about the setup can be found on [this wiki page](https://github.com/RobotCasserole1736/RobotCasserole2017/wiki/Vision-Target-Identification-System).

### Calculators

[GNU Octave](https://www.gnu.org/software/octave/) (aka Matlab) simulations of fuel trajectories, used for initial prototype development.

### logFileSnagger

[Python scripts](https://github.com/RobotCasserole1736/RobotCasserole2017/blob/master/logFileSnagger/file_snagger/snag_files.py) to collect log file from the robot over FTP after matches when we're in the pit. Also includes a script used for uploading files to an AWS server once we get internet connection.

### DataViewer2

A javascript/HTML based viewer of data logs captured from the robot during operation.


