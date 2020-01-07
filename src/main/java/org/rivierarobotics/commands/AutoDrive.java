package org.rivierarobotics.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.DriveTrain;
import org.rivierarobotics.subsystems.DriveTrainSide;

public class AutoDrive extends InstantCommand {
    private final DriveTrain driveTrain;
    private final double distance;

    public AutoDrive(double inches) {
        this.distance = inches;
        this.driveTrain = Robot.runningRobot.driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        DriveTrainSide left = driveTrain.getLeft(), right = driveTrain.getRight();
        left.setPosition(left.getPositionInches() + distance);
        right.setPosition(right.getPositionInches() + distance);
    }
}
