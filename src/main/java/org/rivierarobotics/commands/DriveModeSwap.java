package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.robot.Robot;

public class DriveModeSwap extends InstantCommand {
    @Override
    public void execute() {
        Robot.runningRobot.isArcade = !Robot.runningRobot.isArcade;
    }
}
