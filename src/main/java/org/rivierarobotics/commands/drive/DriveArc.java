package org.rivierarobotics.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import net.octyl.aptcreator.GenerateCreator;
import net.octyl.aptcreator.Provided;
import org.rivierarobotics.subsystems.DriveTrain;

@GenerateCreator
public class DriveArc extends CommandBase {
    private final DriveTrain driveTrain;
    private final double arcRadius;
    private final double time;
    private final double robotWidth = 0.71;
    private double leftVel;
    private double rightVel;
    private double startPos;

    public DriveArc(@Provided DriveTrain driveTrain, double arcRadius, double time) {
        this.driveTrain = driveTrain;
        this.arcRadius = arcRadius;
        this.time = time;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        startPos = driveTrain.getLeft().getPosition();
    }

    @Override
    public void execute() {
        leftVel = (2 * 3.14159 * (arcRadius + (0.5 * robotWidth))) / time;
        rightVel = (2 * 3.14159 * (arcRadius - (0.5 * robotWidth))) / time;
        driveTrain.setVelocity(leftVel, rightVel);
    }

    public DriveArc(@Provided DriveTrain driveTrain, double arcRadius) {
        this(driveTrain, arcRadius, 5);
    }
    @Override
    public void end(boolean interrupted) {
        driveTrain.setPower(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(driveTrain.getLeft().getPosition() - startPos) > Math.abs(2 * 3.14159 * (arcRadius + (0.5 * robotWidth)));
    }
}
