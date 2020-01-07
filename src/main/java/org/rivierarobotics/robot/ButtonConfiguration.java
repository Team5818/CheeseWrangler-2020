package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.AutoDrive;
import org.rivierarobotics.commands.DriveModeSwap;

public class ButtonConfiguration {
    public static void init() {
        JoystickButton changeDriveMode = new JoystickButton(Robot.runningRobot.buttons, 1);
        changeDriveMode.whenPressed(new DriveModeSwap());

        JoystickButton autoForward = new JoystickButton(Robot.runningRobot.buttons, 2);
        autoForward.whenPressed(new AutoDrive(24));
        autoForward.whenReleased(new AutoDrive(-24));
    }
}
