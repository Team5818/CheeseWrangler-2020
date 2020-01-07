# PracticeBot-2020-example
Example code to make a basic practicebot function

## Walkthrough
This code represents the state that your code should be. Please note that it doesn't have to look exactly the same, and should differ as much as you need, so long as it works and adheres to standard convention. As it stands, a robot sporting this code will be able to drive in arcade or tank mode with a standard configuration of talon/spark/spark on each side. It can drive forwards and backwards a certain distance, and can be triggered by a button. All of this is performed under the guise of a Command-based structure adherent to traditional robotics coding practices.

### Packages
You may notice that the code in this repository is nested into different folders under `org/rivierarobotics/`. These are called packages, and help us organize our code. Typically there are four packages: `commands` for WPILib Commands, `robot` for main classes such as Robot and user interfaces, `subsystems` for any WPILib subsystems and their depdendents (e.g. DriveTrain and DriveTrainSide), and `util` for miscellaneous other classes.

### Main/Robot Class
It should be noted that there is no main method present. Instead, the `Robot` class functions as our main method with a method called `autonomousPeriodic()` or `teleopPeriodic()`. These methods are called repeatedly after their `xxxxInit()` method counterparts, and provide the opportunity for the Scheduler (which tells the robot which commands to use) to run. Most commonly in the new WPILib, that can be achieved with `CommandScheduler.getInstance().run()`. The goal of the robot class should be to not have any function code in it. However, for the purposes of this bot, it is advised that you store instances of subsystems in the Robot class. To ensure that only one of each subsystem is created, there should be an instance of Robot created that can be statically called to access the Subsystem instance variables stored within. The Robot class for the purposes of this also stores all the joystick objects as instance variables.

### Commands
Seeing as the structure of this is "Command-based", it's important to understand that a command is equivalent to an instruction for the robot to do something specific, for example move a motor. As such, this is the perfect place to provide function code that does calculations specific to one function. As you can see, I have put the calculations for arcade drive in the `DriveControl` command, which will control the robot for as long as the command is not done. Because the return value of `isFinished()` in that class is always false, that command never stops. To make sure it can actually perform its job, it is crucial that each command that needs a specific subsystem add a requirement for that in the constructor. Typically it is the last line in the constructor, `addRequirements(Subsystem requirements...)`. The command class itself must extend the `CommandBase` class or implement the `Command` interface (READ: the CommandBase option is considerably easier to use).

### Subsystems
As I alluded to earlier, Subsystems are any mechanism that comprises of one or more motors, solenoids, or otherwise controllable output devices. For the scope of FRC, a each mechanism typically gets its own Subsystem. Any subsystem must extend the `SubsystemBase` class or implement the `Subsystem` interface (READ: same as with commands, `SubsystemBase` is much easier to use). The reason we use Subsystems is that they can be easily interfaced with via the command structure. This does not limit you to only specific functions or methods, the range of functions is still the same. However, Subsystems should only contain methods that access directly with the motors in question or their respective PID loops. Try to avoid logic and math in Subsystems.

### Util
The Util package is mostly used for classes that need to be replicated or storage of some sort, as is the case with `RobotMap`. The reason that `MathUtil` is allowed is that it contains functions not only usable in the `commands` package, but potentially elsewhere. 

## Important Notes & Tips
* This uses WPILib 2020, and as such can only be used on robots with the 2020 roborio image and compatible libraries
* This is not the most effective way to write this robot code, it's just a good combination of effective and simple.
* Make sure to read the documentation if you're stuck, or go to the [5818 Wiki](https://github.com/Team5818/Primary). It doesn't have specific information about command-based programming, but more basic projects are covered in depth. It should be noted that the explanation of Commands is now out of date. Refer to the official WPILib documentation (and Java API documentation) for a better explanation of that.
* Try out your code on the practicebot. If it doesn't work, diagnose the problem by experimenting and pushing new versions to the robot. The goal is to get familiar with WPILib, command-based programming, and troubleshooting robots.
* Don't get hung up over formatting, but keep it to a reasonable level of indenting and curly bracket placement. READ: DO NOT place open curly brackets a line below the header, consistency is key with that.
* Don't overcomplicate your code - often more simple code is better.