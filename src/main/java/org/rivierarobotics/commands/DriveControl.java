package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;

public class DriveControl extends CommandBase {
    private final DriveTrain driveTrain;
    private final Joystick leftJs, rightJs;

    public DriveControl() {
        this.driveTrain = Robot.runningRobot.driveTrain;
        this.leftJs = Robot.runningRobot.leftJs;
        this.rightJs = Robot.runningRobot.rightJs;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        if(Robot.runningRobot.isArcade) {
            double left, right;
            double x = leftJs.getX(), y = leftJs.getY();
            if (y >= 0) {
                left = y+x;
                right = y-x;
            } else {
                left = y-x;
                right = y+x;
            }
            driveTrain.setPower(left, right);
        } else {
            driveTrain.setPower(leftJs.getY(), rightJs.getY());
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
